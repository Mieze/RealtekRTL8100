/* RealtekRTL8100.c -- RTL8100 driver class implementation.
 *
 * Copyright (c) 2013 Laura Müller <laura-mueller@uni-duesseldorf.de>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * Driver for Realtek RTL8100x PCIe ethernet controllers.
 *
 * This driver is based on Realtek's r8101 Linux driver (1.030.02).
 */


#include "RealtekRTL8100.h"

#pragma mark --- function prototypes ---

static inline UInt32 adjustIPv6Header(mbuf_t m);

static inline u32 ether_crc(int length, unsigned char *data);

#pragma mark --- public methods ---

OSDefineMetaClassAndStructors(RTL8100, super)

/* IOService (or its superclass) methods. */

bool RTL8100::init(OSDictionary *properties)
{
    bool result;
    
    result = super::init(properties);
    
    if (result) {
        workLoop = NULL;
        commandGate = NULL;
        pciDevice = NULL;
        mediumDict = NULL;
        txQueue = NULL;
        interruptSource = NULL;
        timerSource = NULL;
        netif = NULL;
        netStats = NULL;
        etherStats = NULL;
        baseMap = NULL;
        baseAddr = NULL;
        rxMbufCursor = NULL;
        txNext2FreeMbuf = NULL;
        txMbufCursor = NULL;
        statBufDesc = NULL;
        statPhyAddr = NULL;
        statData = NULL;
        isEnabled = false;
        promiscusMode = false;
        multicastMode = false;
        linkUp = false;
        rxPoll = false;
        polling = false;
        mtu = ETH_DATA_LEN;
        powerState = 0;
        speed = SPEED_1000;
        duplex = DUPLEX_FULL;
        autoneg = AUTONEG_ENABLE;
        flowCtl = kFlowControlOff;
        eeeAdv = 0;
        eeeCap = 0;
        pciDeviceData.vendor = 0;
        pciDeviceData.device = 0;
        pciDeviceData.subsystem_vendor = 0;
        pciDeviceData.subsystem_device = 0;
        linuxData.pci_dev = &pciDeviceData;
        intrMitigateValue = 0;
        wolCapable = false;
        wolActive = false;
        enableTSO4 = false;
        enableTSO6 = false;
        enableCSO6 = false;
        disableASPM = false;
        enableEEE = false;
    }
    
done:
    return result;
}

void RTL8100::free()
{
    UInt32 i;
    
    DebugLog("free() ===>\n");
    
    if (workLoop) {
        if (interruptSource) {
            workLoop->removeEventSource(interruptSource);
            RELEASE(interruptSource);
        }
        if (timerSource) {
            workLoop->removeEventSource(timerSource);
            RELEASE(timerSource);
        }
        workLoop->release();
        workLoop = NULL;
    }
    RELEASE(commandGate);
    RELEASE(txQueue);
    RELEASE(mediumDict);
    
    for (i = MEDIUM_INDEX_AUTO; i < MEDIUM_INDEX_COUNT; i++)
        mediumTable[i] = NULL;
    
    RELEASE(baseMap);
    baseAddr = NULL;
    linuxData.mmio_addr = NULL;
    
    RELEASE(pciDevice);
    freeDMADescriptors();
    
    DebugLog("free() <===\n");
    
    super::free();
}

static const char *onName = "enabled";
static const char *offName = "disabled";

/*! @function start
     @abstract Starts the network controller.
     @discussion After the controller driver has successfully matched
     to a provider, this method is called to start the network controller.
     IONetworkController will allocate resources and gather controller
     properties in its implementation. No I/O will be performed until
     the subclass tries to attach a client object. A driver must override
     this method, and call super::start() at the beginning of its own
     implementation. Then check the return value to make sure that its
     superclass was started successfully before proceeding. Tasks that
     are usually performed by a driver's start method are: resource
     allocation, hardware initialization, allocation of IOEventSources
     and attaching them to a workloop, publishing a medium dictionary,
     and finally, attaching an interface object when it is ready to
     handle client requests.
     @param provider The provider that the controller was matched
     (and attached) to.
     @result Returns true on success, false otherwise.
 */

bool RTL8100::start(IOService *provider)
{
    bool result;
    
    result = super::start(provider);
    
    if (!result) {
        IOLog("Ethernet [RealtekRTL8100]: IOEthernetController::start failed.\n");
        goto done;
    }
    multicastMode = false;
    promiscusMode = false;
    multicastFilter = 0;
    
    pciDevice = OSDynamicCast(IOPCIDevice, provider);
    
    if (!pciDevice) {
        IOLog("Ethernet [RealtekRTL8100]: No provider.\n");
        goto done;
    }
    pciDevice->retain();
    
    if (!pciDevice->open(this)) {
        IOLog("Ethernet [RealtekRTL8100]: Failed to open provider.\n");
        goto error1;
    }
    getParams();

    if (!initPCIConfigSpace(pciDevice)) {
        goto error2;
    }
    if (!initRTL8100()) {
        goto error2;
    }
    
    if (!setupMediumDict()) {
        IOLog("Ethernet [RealtekRTL8100]: Failed to setup medium dictionary.\n");
        goto error2;
    }
    commandGate = getCommandGate();
    
    if (!commandGate) {
        IOLog("Ethernet [RealtekRTL8100]: getCommandGate() failed.\n");
        goto error3;
    }
    commandGate->retain();
    
    if (!initEventSources(provider)) {
        IOLog("Ethernet [RealtekRTL8100]: initEventSources() failed.\n");
        goto error3;
    }
    
    result = attachInterface(reinterpret_cast<IONetworkInterface**>(&netif));
    
    if (!result) {
        IOLog("Ethernet [RealtekRTL8100]: attachInterface() failed.\n");
        goto error3;
    }
    pciDevice->close(this);
    result = true;
    
done:
    return result;
    
error3:
    RELEASE(commandGate);
    
error2:
    pciDevice->close(this);
    
error1:
    pciDevice->release();
    pciDevice = NULL;
    goto done;
}

/*! @function stop
     @abstract Stops the network controller.
     @discussion The counterpart of start(). The controller has been
     instructed to stop running. The stop() method should release
     resources and undo actions performed by the start() method.
     Subclasses must override this method and call super::stop()
     at the end of its implementation.
     @param provider The provider that the controller was matched
     (and attached) to. */

void RTL8100::stop(IOService *provider)
{
    UInt32 i;
    
    if (netif) {
        detachInterface(netif);
        netif = NULL;
    }
    if (workLoop) {
        if (interruptSource) {
            workLoop->removeEventSource(interruptSource);
            RELEASE(interruptSource);
        }
        if (timerSource) {
            workLoop->removeEventSource(timerSource);
            RELEASE(timerSource);
        }
        workLoop->release();
        workLoop = NULL;
    }
    RELEASE(commandGate);
    RELEASE(txQueue);
    RELEASE(mediumDict);
    
    for (i = MEDIUM_INDEX_AUTO; i < MEDIUM_INDEX_COUNT; i++)
        mediumTable[i] = NULL;
    
    freeDMADescriptors();
    RELEASE(baseMap);
    baseAddr = NULL;
    linuxData.mmio_addr = NULL;
    
    RELEASE(pciDevice);
    
    super::stop(provider);
}

/* Power Management Support */
static IOPMPowerState powerStateArray[kPowerStateCount] =
{
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {1, kIOPMDeviceUsable, kIOPMPowerOn, kIOPMPowerOn, 0, 0, 0, 0, 0, 0, 0, 0}
};

/*! @function registerWithPolicyMaker
     @abstract Implemented by controller drivers to register with
     the power management policy-maker.
     @discussion Drivers that are able to power manage their hardware
     should override this method and register with the policy-maker
     provided by calling IOService::registerPowerDriver().
     IONetworkController will call this method before the initial
     attempt is made to attach a client.
     @param policyMaker The policy-maker chosen to manage power for
     this network controller.
     @result Returns kIOReturnSuccess on success, kIOReturnUnsupported if the
     driver does not support power management, or an appropriate error
     return code. The default return is kIOReturnUnsupported. */

IOReturn RTL8100::registerWithPolicyMaker(IOService *policyMaker)
{
    DebugLog("registerWithPolicyMaker() ===>\n");
    
    powerState = kPowerStateOn;
    
    DebugLog("registerWithPolicyMaker() <===\n");
    
    return policyMaker->registerPowerDriver(this, powerStateArray, kPowerStateCount);
}


/*! @function setPowerState
     @abstract Requests a power managed driver to change the power state of its device.
     @discussion A power managed driver must override <code>setPowerState</code> to take part in system power management. After a driver is registered with power management, the system uses <code>setPowerState</code> to power the device off and on for system sleep and wake.
     Calls to @link PMinit PMinit@/link and @link registerPowerDriver registerPowerDriver@/link enable power management to change a device's power state using <code>setPowerState</code>. <code>setPowerState</code> is called in a clean and separate thread context.
     @param powerStateOrdinal The number in the power state array of the state the driver is being instructed to switch to.
     @param whatDevice A pointer to the power management object which registered to manage power for this device. In most cases, <code>whatDevice</code> will be equal to your driver's own <code>this</code> pointer.
     @result The driver must return <code>IOPMAckImplied</code> if it has complied with the request when it returns. Otherwise if it has started the process of changing power state but not finished it, the driver should return a number of microseconds which is an upper limit of the time it will need to finish. Then, when it has completed the power switch, it should call @link acknowledgeSetPowerState acknowledgeSetPowerState@/link. */

IOReturn RTL8100::setPowerState(unsigned long powerStateOrdinal, IOService *policyMaker)
{
    IOReturn result = IOPMAckImplied;
    
    DebugLog("setPowerState() ===>\n");
    
    if (powerStateOrdinal == powerState) {
        DebugLog("Ethernet [RealtekRTL8100]: Already in power state %lu.\n", powerStateOrdinal);
        goto done;
    }
    DebugLog("Ethernet [RealtekRTL8100]: switching to power state %lu.\n", powerStateOrdinal);
    
    if (powerStateOrdinal == kPowerStateOff)
        commandGate->runAction(setPowerStateSleepAction);
    else
        commandGate->runAction(setPowerStateWakeAction);
    
    powerState = powerStateOrdinal;
    
done:
    DebugLog("setPowerState() <===\n");
    
    return result;
}

/*! @function systemWillShutdown
     @abstract Handles system shutdown and restart notifications.
     @discussion Overrides <code>IOService::systemWillShutdown</code> in order
     to notify network clients that the power-managed controller should be disabled.
     As a result, drivers can expect their <code>disable</code> method to be called
     before system shutdown or restart. This implementation is synchronous and can
     block before calling <code>IOService::systemWillShutdown</code> and return.
     @param specifier
        <code>kIOMessageSystemWillPowerOff</code> or <code>kIOMessageSystemWillRestart</code>.
     @see //apple_ref/cpp/instm/IOService/systemWillShutdown/void/(IOOptionBits) IOService::systemWillShutdown */

void RTL8100::systemWillShutdown(IOOptionBits specifier)
{
    DebugLog("systemWillShutdown() ===>\n");
    
    if ((kIOMessageSystemWillPowerOff | kIOMessageSystemWillRestart) & specifier)
        disable(netif);
    
    /* Restore the original MAC address. */
    rtl8101_rar_set(&linuxData, (UInt8 *)&origMacAddr.bytes);

    DebugLog("systemWillShutdown() <===\n");
    
    /* Must call super shutdown or system will stall. */
    super::systemWillShutdown(specifier);
}

/* IONetworkController methods. */

/*! @function enable
 @abstract A request from an interface client to enable the controller.
     @discussion This method is called by an interface client to enable the controller.
     Upon receiving this command, the controller driver must bring up the
     hardware and become ready to transmit and receive packets. A driver
     should also delay the allocation of most runtime resources until this
     method is called in order to conserve system resources. This method call
     is synchronized by the workloop's gate.
     @param interface The interface client object that requested the enable.
     @result Returns kIOReturnUnsupported. Drivers that override this method must
     return kIOReturnSuccess on success, or an error code otherwise.
 */

IOReturn RTL8100::enable(IONetworkInterface *netif)
{
    const IONetworkMedium *selectedMedium;
    IOReturn result = kIOReturnError;
    
    DebugLog("enable() ===>\n");
    
    if (isEnabled) {
        DebugLog("Ethernet [RealtekRTL8100]: Interface already enabled.\n");
        result = kIOReturnSuccess;
        goto done;
    }
    if (!pciDevice || pciDevice->isOpen()) {
        IOLog("Ethernet [RealtekRTL8100]: Unable to open PCI device.\n");
        goto done;
    }
    pciDevice->open(this);
    
    if (!setupDMADescriptors()) {
        IOLog("Ethernet [RealtekRTL8100]: Error allocating DMA descriptors.\n");
        goto done;
    }
    selectedMedium = getSelectedMedium();
    
    if (!selectedMedium) {
        DebugLog("Ethernet [RealtekRTL8100]: No medium selected. Falling back to autonegotiation.\n");
        selectedMedium = mediumTable[MEDIUM_INDEX_AUTO];
    }
    selectMedium(selectedMedium);
    setLinkStatus(kIONetworkLinkValid);
    enableRTL8100();
    
    /* In case we are using an msi the interrupt hasn't been enabled by start(). */
    interruptSource->enable();
    
    txDescDoneCount = txDescDoneLast = 0;
    deadlockWarn = 0;
    needsUpdate = false;
    isEnabled = true;
    polling = false;
    
    timerSource->setTimeoutMS(kTimeoutMS);
    
    result = kIOReturnSuccess;
    
    DebugLog("enable() <===\n");
    
done:
    return result;
}

/*! @function disable
     @abstract A request from an interface client to disable the controller.
     @discussion This method is called by an interface client to disable the controller.
     This method should stop the hardware and disable hardware interrupt
     sources. Any resources allocated by enable() should also be deallocated.
     This method call is synchronized by the workloop's gate.
     @param interface The interface object that requested the disable.
     @result kIOReturnUnsupported. Drivers that override this method must
     return Returns kIOReturnSuccess on success, or an error code otherwise.
 */

IOReturn RTL8100::disable(IONetworkInterface *netif)
{
    IOReturn result = kIOReturnSuccess;
    
    DebugLog("disable() ===>\n");
    
    if (!isEnabled)
        goto done;
    
    netif->stopOutputThread();
    netif->flushOutputQueue();
    
    polling = false;
    isEnabled = false;
    
    timerSource->cancelTimeout();
    needsUpdate = false;
    txDescDoneCount = txDescDoneLast = 0;
    
    /* In case we are using msi disable the interrupt. */
    interruptSource->disable();
    
    disableRTL8100();
    
    setLinkStatus(kIONetworkLinkValid);
    linkUp = false;
    txClearDescriptors();
    
    if (pciDevice && pciDevice->isOpen())
        pciDevice->close(this);
    
    freeDMADescriptors();
    
    DebugLog("disable() <===\n");
    
done:
    return result;
}

/*! @function outputStart
        @abstract An indication to the driver to dequeue and transmit packets
     waiting in the interface output queue.
        @discussion A driver that supports the pull output model must override this
     method, which will be called by a per-interface output thread when a packet
     is added to the interface output queue. In response, driver must verify that
     free transmit resources are available, then dequeue one or more packets by
     calling <code>IONetworkInterface::dequeueOutputPackets()</code>. Packets
     removed from the queue are owned by the driver, and should be immediately
     prepared for transmission. Additional software queueing at the driver layer
     to store the dequeued packets for delayed transmission is highly discouraged
     unless absolutely necessary. If transmit resources are exhausted, the driver
     should quickly return <code>kIOReturnNoResources</code> to force the output
     thread to retry later, otherwise the output thread will continue to call
     this method until the output queue is empty. When driver creates a single
     network interface, this method will execute in a single threaded context.
     However, it is the driver's responsibility to protect transmit resources
     that are shared with other driver threads. To simplify drivers that wish to
     process output packets on their work loop context, the family provides an
     option to force the output thread to always call this method through a
     <code>runAction()</code>. However this can have negative performance
     implications due to extra locking and serializing the output thread against
     other work loop events. Another option that drivers can deploy to
     synchronize against the output thread is to issue a thread stop before
     touching any shared resources. But this should be used sparingly on the
     data path since stopping the output thread can block.
     @param interface The network interface with packet(s) to transmit.
     @param options Always zero.
     @result <code>kIOReturnSuccess</code> on success, output thread will
     continue calling the driver until the output queue is empty.
     <code>kIOReturnNoResources</code> when there is a temporary driver resource
     shortage.
 */

IOReturn RTL8100::outputStart(IONetworkInterface *interface, IOOptionBits options )
{
    IOPhysicalSegment txSegments[kMaxSegs];
    mbuf_t m;
    RtlDmaDesc *desc, *firstDesc;
    IOReturn result = kIOReturnNoResources;
    UInt32 cmd;
    UInt32 opts2;
    mbuf_tso_request_flags_t tsoFlags;
    mbuf_csum_request_flags_t checksums;
    UInt32 mssValue;
    UInt32 opts1;
    UInt32 vlanTag;
    UInt32 numSegs;
    UInt32 lastSeg;
    UInt32 index;
    UInt32 i;
    
    //DebugLog("outputStart() ===>\n");
    
    if (!(isEnabled && linkUp)) {
        DebugLog("Ethernet [RealtekRTL8100]: Interface down. Dropping packets.\n");
        goto done;
    }
    while ((txNumFreeDesc > (kMaxSegs + 3)) && (interface->dequeueOutputPackets(1, &m, NULL, NULL, NULL) == kIOReturnSuccess)) {
        cmd = 0;
        opts2 = 0;
        
        if (mbuf_get_tso_requested(m, &tsoFlags, &mssValue)) {
            DebugLog("Ethernet [RealtekRTL8100]: mbuf_get_tso_requested() failed. Dropping packet.\n");
            freePacket(m);
            continue;
        }
        if (tsoFlags & (MBUF_TSO_IPV4 | MBUF_TSO_IPV6)) {
            if (tsoFlags & MBUF_TSO_IPV4) {
                getTso4Command(&cmd, &opts2, mssValue, tsoFlags);
            } else {
                /* The pseudoheader checksum has to be adjusted first. */
                adjustIPv6Header(m);
                getTso6Command(&cmd, &opts2, mssValue, tsoFlags);
            }
        } else {
            /* We use mssValue as a dummy here because it isn't needed anymore. */
            mbuf_get_csum_requested(m, &checksums, &mssValue);
            getChecksumCommand(&cmd, &opts2, checksums);
        }
        /* Finally get the physical segments. */
        numSegs = txMbufCursor->getPhysicalSegmentsWithCoalesce(m, &txSegments[0], kMaxSegs);
        
        /* Alloc required number of descriptors. As the descriptor which has been freed last must be
         * considered to be still in use we never fill the ring completely but leave at least one
         * unused.
         */
        if (!numSegs) {
            DebugLog("Ethernet [RealtekRTL8100]: getPhysicalSegmentsWithCoalesce() failed. Dropping packet.\n");
            freePacket(m);
            continue;
        }
        OSAddAtomic(-numSegs, &txNumFreeDesc);
        index = txNextDescIndex;
        txNextDescIndex = (txNextDescIndex + numSegs) & kTxDescMask;
        firstDesc = &txDescArray[index];
        lastSeg = numSegs - 1;
        
        /* Next fill in the VLAN tag. */
        opts2 |= (getVlanTagDemand(m, &vlanTag)) ? (OSSwapInt16(vlanTag) | TxVlanTag) : 0;
        
        /* And finally fill in the descriptors. */
        for (i = 0; i < numSegs; i++) {
            desc = &txDescArray[index];
            opts1 = (((UInt32)txSegments[i].length) | cmd);
            opts1 |= (i == 0) ? FirstFrag : DescOwn;
            
            if (i == lastSeg) {
                opts1 |= LastFrag;
                txMbufArray[index] = m;
            } else {
                txMbufArray[index] = NULL;
            }
            if (index == kTxLastDesc)
                opts1 |= RingEnd;
            
            desc->addr = OSSwapHostToLittleInt64(txSegments[i].location);
            desc->opts2 = OSSwapHostToLittleInt32(opts2);
            desc->opts1 = OSSwapHostToLittleInt32(opts1);
            
            //DebugLog("opts1=0x%x, opts2=0x%x, addr=0x%llx, len=0x%llx\n", opts1, opts2, txSegments[i].location, txSegments[i].length);
            ++index &= kTxDescMask;
        }
        firstDesc->opts1 |= DescOwn;
    }
    /* Set the polling bit. */
    WriteReg8(TxPoll, NPQ);
    
    result = (txNumFreeDesc > (kMaxSegs + 3)) ? kIOReturnSuccess : kIOReturnNoResources;
    
done:
    //DebugLog("outputStart() <===\n");
    
    return result;
}

/*! @function getPacketBufferConstraints
     @abstract Gets the controller's packet buffer constraints.
     @discussion Called by start() to obtain the constraints on the
     memory buffer for each mbuf packet allocated through allocatePacket().
     Drivers can override this method to specify the buffer constraints
     imposed by their bus master hardware. Note that outbound packets,
     those that originate from the network stack, are not currently
     subject to the constraints reported here.
     @param constraints A pointer to an IOPacketBufferConstraints
     structure that this method is expected to initialize.
     See IOPacketBufferConstraints structure definition.
 */

void RTL8100::getPacketBufferConstraints(IOPacketBufferConstraints *constraints) const
{
    DebugLog("getPacketBufferConstraints() ===>\n");
    
	constraints->alignStart = kIOPacketBufferAlign8;
	constraints->alignLength = kIOPacketBufferAlign8;
    
    DebugLog("getPacketBufferConstraints() <===\n");
}

/*! @function createOutputQueue
     @abstract Creates an IOOutputQueue to handle output packet queueing,
     and also to resolve contention for the controller's transmitter from
     multiple client threads.
     @discussion This method is called by start() to create an IOOutputQueue object to
     handle output packet queueing. The default implementation will always
     return 0, hence no output queue will be created. A driver may override
     this method and return a subclass of IOOutputQueue. IONetworkController
     will keep a reference to the queue created, and will release this
     object when IONetworkController is freed. Also see getOutputQueue().
     @result Returns a newly allocated and initialized IOOutputQueue object.
 */

IOOutputQueue* RTL8100::createOutputQueue()
{
    DebugLog("createOutputQueue() ===>\n");
    
    DebugLog("createOutputQueue() <===\n");
    
    return IOBasicOutputQueue::withTarget(this);
}

/*! @function newVendorString
     @result Returns a string describing the vendor of the network controller.
     The caller is responsible for releasing the string object returned. */

const OSString* RTL8100::newVendorString() const
{
    DebugLog("newVendorString() ===>\n");
    
    DebugLog("newVendorString() <===\n");
    
    return OSString::withCString("Realtek");
}

/*! @function newModelString
     @result Returns a string describing the model of the network controller.
     The caller is responsible for releasing the string object returned. */

const OSString* RTL8100::newModelString() const
{
    DebugLog("newModelString() ===>\n");
    DebugLog("newModelString() <===\n");
    
    return OSString::withCString(rtl_chip_info[linuxData.chipset].name);
}

/*! @function configureInterface
     @abstract Configures a newly created network interface object.
     @discussion This method configures an interface object that was created by
     createInterface(). Subclasses can override this method to customize
     and examine the interface object that will be attached to the
     controller as a client.
     @param interface The interface object to be configured.
     @result Returns true if the operation was successful, false otherwise
     (this will cause attachInterface() to fail and return 0).
 */

bool RTL8100::configureInterface(IONetworkInterface *interface)
{
    char modelName[kNameLenght];
    IONetworkData *data;
    IOReturn error;
    bool result;
    
    DebugLog("configureInterface() ===>\n");
    
    result = super::configureInterface(interface);
    
    if (!result)
        goto done;
	
    /* Get the generic network statistics structure. */
    data = interface->getParameter(kIONetworkStatsKey);
    
    if (data) {
        netStats = (IONetworkStats *)data->getBuffer();
        
        if (!netStats) {
            IOLog("Ethernet [RealtekRTL8100]: Error getting IONetworkStats\n.");
            result = false;
            goto done;
        }
    }
    /* Get the Ethernet statistics structure. */
    data = interface->getParameter(kIOEthernetStatsKey);
    
    if (data) {
        etherStats = (IOEthernetStats *)data->getBuffer();
        
        if (!etherStats) {
            IOLog("Ethernet [RealtekRTL8100]: Error getting IOEthernetStats\n.");
            result = false;
            goto done;
        }
    }
    /* Enable support for the new network driver interface with packet scheduling. */
    error = interface->configureOutputPullModel(512, 0, 0, IONetworkInterface::kOutputPacketSchedulingModelNormal);
    
    if (error != kIOReturnSuccess) {
        IOLog("Ethernet [RealtekRTL8100]: configureOutputPullModel() failed\n.");
        result = false;
        goto done;
    }
    /* Enable support for polled receive mode. */
    if (rxPoll) {
        error = interface->configureInputPacketPolling(kNumRxDesc, kIONetworkWorkLoopSynchronous);
        
        if (error != kIOReturnSuccess) {
            IOLog("Ethernet [RealtekRTL8100]: configureInputPacketPolling() failed\n.");
            result = false;
            goto done;
        }
    }
    snprintf(modelName, kNameLenght, "Realtek %s PCI Express Fast Ethernet", rtl_chip_info[linuxData.chipset].name);
    setProperty("model", modelName);
    
    DebugLog("configureInterface() <===\n");
    
done:
    return result;
}

/*! @function createWorkLoop
     @abstract Method called by IONetworkController prior to the initial
     getWorkLoop() call.
     @discussion Before IONetworkController calls getWorkLoop() in its
     start() method, it will call createWorkLoop() to make sure that a
     subclass that wants to create a workloop, will do so before its
     first use.
     @result Returns true to indicate success, false otherwise. Returning false
     will fail IONetworkController::start().
*/

bool RTL8100::createWorkLoop()
{
    DebugLog("createWorkLoop() ===>\n");
    
    workLoop = IOWorkLoop::workLoop();
    
    DebugLog("createWorkLoop() <===\n");
    
    return workLoop ? true : false;
}

/*! @function getWorkLoop
     @abstract Returns the current work loop or <code>provider->getWorkLoop</code>.
     @discussion This function returns a valid work loop that a client can use to add an IOCommandGate to. The intention is that an IOService client has data that needs to be protected but doesn't want to pay the cost of a dedicated thread. This data has to be accessed from a provider's call-out context as well. So to achieve both of these goals the client creates an IOCommandGate to lock access to his data but he registers it with the provider's work loop, i.e. the work loop which will make the completion call-outs. This avoids a potential deadlock because the work loop gate uses a recursive lock, which allows the same lock to be held multiple times by a single thread.
     @result A work loop, either the current work loop or it walks up the @link getProvider getProvider@/link chain calling <code>getWorkLoop</code>. Eventually it will reach a valid work loop-based driver or the root of the I/O tree, where it will return a system-wide work loop. Returns 0 if it fails to find (or create) a work loop.*/

IOWorkLoop* RTL8100::getWorkLoop() const
{
    DebugLog("getWorkLoop() ===>\n");
    
    DebugLog("getWorkLoop() <===\n");
    
    return workLoop;
}

/* Methods inherited from IOEthernetController. */

/*! @function getHardwareAddress
     @abstract Gets the Ethernet controller's permanent station address.
     @discussion Ethernet drivers must implement this method, by reading the
     address from hardware and writing it to the buffer provided. This method
     is called from the workloop context.
     @param addrP Pointer to an IOEthernetAddress where the hardware address
     should be returned.
     @result Returns kIOReturnSuccess on success, or an error return code otherwise.
*/

IOReturn RTL8100::getHardwareAddress(IOEthernetAddress *addr)
{
    IOReturn result = kIOReturnError;
    
    DebugLog("getHardwareAddress() ===>\n");
    
    if (addr) {
        bcopy(&currMacAddr.bytes, addr->bytes, kIOEthernetAddressSize);
        result = kIOReturnSuccess;
    }
    
    DebugLog("getHardwareAddress() <===\n");
    
    return result;
}

/*! @function setHardwareAddress
     @abstract Sets or changes the station address used by the Ethernet
     controller.
     @discussion This method is called in response to a client command to
     change the station address used by the Ethernet controller. Implementation
     of this method is optional. This method is called from the workloop context.
     @param addrP Pointer to an IOEthernetAddress containing the new station
     address.
     @result The default implementation will always return kIOReturnUnsupported.
     If overridden, drivers must return kIOReturnSuccess on success, or an error
     return code otherwise.
*/

IOReturn RTL8100::setHardwareAddress(const IOEthernetAddress *addr)
{
    IOReturn result = kIOReturnError;
    
    DebugLog("setHardwareAddress() ===>\n");
    
    if (addr) {
        bcopy(addr->bytes, &currMacAddr.bytes, kIOEthernetAddressSize);
        rtl8101_rar_set(&linuxData, (UInt8 *)&currMacAddr.bytes);
        result = kIOReturnSuccess;
    }
    
    DebugLog("setHardwareAddress() <===\n");
    
    return result;
}

/*! @function setPromiscuousMode
     @abstract Enables or disables promiscuous mode.
     @discussion Called by enablePacketFilter() or disablePacketFilter()
     when there is a change in the activation state of the promiscuous
     filter identified by kIOPacketFilterPromiscuous. This method is
     called from the workloop context.
     @param active True to enable promiscuous mode, false to disable it.
     @result Returns kIOReturnUnsupported. If overridden, drivers must return
     kIOReturnSuccess on success, or an error return code otherwise.
*/

IOReturn RTL8100::setPromiscuousMode(bool active)
{
    UInt32 *filterAddr = (UInt32 *)&multicastFilter;
    UInt32 mcFilter[2];
    UInt32 rxMode;
    
    DebugLog("setPromiscuousMode() ===>\n");
    
    if (active) {
        DebugLog("Ethernet [RealtekRTL8100]: Promiscuous mode enabled.\n");
        rxMode = (AcceptBroadcast | AcceptMulticast | AcceptMyPhys | AcceptAllPhys);
        mcFilter[1] = mcFilter[0] = 0xffffffff;
    } else {
        DebugLog("Ethernet [RealtekRTL8100]: Promiscuous mode disabled.\n");
        rxMode = (AcceptBroadcast | AcceptMulticast | AcceptMyPhys);
        mcFilter[0] = *filterAddr++;
        mcFilter[1] = *filterAddr;
    }
    promiscusMode = active;
    rxMode |= (ReadReg32(RxConfig) & rxConfigMask);
    WriteReg32(RxConfig, rxMode);
    WriteReg32(MAR0, mcFilter[0]);
    WriteReg32(MAR1, mcFilter[1]);
    
    DebugLog("setPromiscuousMode() <===\n");
    
    return kIOReturnSuccess;
}

/*! @function setMulticastMode
     @abstract Enables or disables multicast mode.
     @discussion Called by enablePacketFilter() or disablePacketFilter()
     when there is a change in the activation state of the multicast filter
     identified by kIOPacketFilterMulticast. This method is called from the
     workloop context.
     @param active True to enable multicast mode, false to disable it.
     @result Returns kIOReturnUnsupported. If overridden, drivers must return
     kIOReturnSuccess on success, or an error return code otherwise.
*/

IOReturn RTL8100::setMulticastMode(bool active)
{
    UInt32 *filterAddr = (UInt32 *)&multicastFilter;
    UInt32 mcFilter[2];
    UInt32 rxMode;
    
    DebugLog("setMulticastMode() ===>\n");
    
    if (active) {
        rxMode = (AcceptBroadcast | AcceptMulticast | AcceptMyPhys);
        mcFilter[0] = *filterAddr++;
        mcFilter[1] = *filterAddr;
    } else{
        rxMode = (AcceptBroadcast | AcceptMyPhys);
        mcFilter[1] = mcFilter[0] = 0;
    }
    multicastMode = active;
    rxMode |= (ReadReg32(RxConfig) & rxConfigMask);
    WriteReg32(RxConfig, rxMode);
    WriteReg32(MAR0, mcFilter[0]);
    WriteReg32(MAR1, mcFilter[1]);
    
    DebugLog("setMulticastMode() <===\n");
    
    return kIOReturnSuccess;
}

/*! @function setMulticastList
     @abstract Sets the list of multicast addresses a multicast filter
     should use to match against the destination address of an incoming frame.
     @discussion This method sets the list of multicast addresses that the multicast filter
     should use to match against the destination address of an incoming frame.
     The frame should be accepted when a match occurs.  Called when the multicast group membership of an interface
     object is changed. Drivers that support kIOPacketFilterMulticast should
     override this method and update the hardware multicast filter using the
     list of Ethernet addresses provided. Perfect multicast filtering is
     preferred if supported by the hardware, in order to reduce the number of
     unwanted packets received. If the number of multicast addresses in the
     list exceeds what the hardware is capable of supporting, or if perfect
     filtering is not supported, then ideally the hardware should be programmed
     to perform imperfect filtering, through some form of hash filtering
     mechanism. Only as a last resort should the driver enable reception of
     all multicast packets to satisfy this request. This method is called
     from the workloop context, and only if the driver reports
     kIOPacketFilterMulticast support in getPacketFilters().
     @param addrs An array of Ethernet addresses. This argument must be
     ignored if the count argument is 0.
     @param count The number of Ethernet addresses in the list. This value
     will be zero when the list becomes empty.
     @result Returns kIOReturnUnsupported. Drivers must return kIOReturnSuccess to
     indicate success, or an error return code otherwise.
*/

IOReturn RTL8100::setMulticastList(IOEthernetAddress *addrs, UInt32 count)
{
    UInt32 *filterAddr = (UInt32 *)&multicastFilter;
    UInt64 filter = 0;
    UInt32 i, bitNumber;
    
    DebugLog("setMulticastList() ===>\n");
    
    if (count <= kMCFilterLimit) {
        for (i = 0; i < count; i++, addrs++) {
            bitNumber = ether_crc(6, reinterpret_cast<unsigned char *>(addrs)) >> 26;
            filter |= (1 << (bitNumber & 0x3f));
        }
        multicastFilter = OSSwapInt64(filter);
    } else {
        multicastFilter = 0xffffffffffffffff;
    }
    WriteReg32(MAR0, *filterAddr++);
    WriteReg32(MAR1, *filterAddr);
    
    DebugLog("setMulticastList() <===\n");
    
    return kIOReturnSuccess;
}

/*! @function getChecksumSupport
     @abstract Gets checksums that are supported by the network controller for
     the given checksum family.
     @discussion A network controller that is capable of inserting and verifying
     checksums on output and input packets, should override this method and
     advertise its capability in order to assist or offload the software checksum
     calculations performed by the protocol stacks.
     @param checksumMask A pointer to the mask of supported checksums returned
     by this method.
     @param checksumFamily A value that specifies the checksum family.
     @param isOutput Set to true to query the support for checksum insertion on
     output packets, or false to query the support for checksum verification
     on input packets. Controllers that have symmetric hardware checksum support
     can return a fixed checksum mask value, and ignore this argument.
     @result Default return is kIOReturnUnsupported. Controllers that override
     this method must return kIOReturnSuccess. Any other return value will be
     interpretated as a lack of checksum support, regardless of the value
     returned through the first argument.
*/

IOReturn RTL8100::getChecksumSupport(UInt32 *checksumMask, UInt32 checksumFamily, bool isOutput)
{
    IOReturn result = kIOReturnUnsupported;
    
    DebugLog("getChecksumSupport() ===>\n");
    
    if ((checksumFamily == kChecksumFamilyTCPIP) && checksumMask) {
        if (isOutput) {
            if (revision2)
                *checksumMask = (enableCSO6) ? (kChecksumTCP | kChecksumUDP | kChecksumIP | kChecksumTCPIPv6 | kChecksumUDPIPv6) : (kChecksumTCP | kChecksumUDP | kChecksumIP);
            else
                *checksumMask = (kChecksumTCP | kChecksumUDP | kChecksumIP);
        } else {
            *checksumMask = (revision2) ? (kChecksumTCP | kChecksumUDP | kChecksumIP | kChecksumTCPIPv6 | kChecksumUDPIPv6) : (kChecksumTCP | kChecksumUDP | kChecksumIP);
        }
        result = kIOReturnSuccess;
    }
    DebugLog("getChecksumSupport() <===\n");
    
    return result;
}

/*! @function setMaxPacketSize
     @abstract A client request to change the maximum packet size.
     @discussion This method call is synchronized by the workloop's gate.
     @param maxSize The new maximum packet size.
     @result Returns kIOReturnUnsupported. Drivers may override this method
     and return either kIOReturnSuccess to indicate that the new size
     was accepted and is in effect, or an error code to indicate failure.
*/

IOReturn RTL8100::setMaxPacketSize (UInt32 maxSize)
{
    IOReturn result = kIOReturnUnsupported;
    
done:
    return result;
}

/*! @function getMaxPacketSize
     @abstract Gets the maximum packet size supported by the Ethernet
     controller, including the frame header and FCS.
     @param maxSize Pointer to the return value.
     @result Returns kIOReturnSuccess on success, or an error code otherwise.
*/

IOReturn RTL8100::getMaxPacketSize (UInt32 *maxSize) const
{
    IOReturn result = kIOReturnBadArgument;
    
    if (maxSize) {
        *maxSize = mtu + ETHER_HDR_LEN + ETHER_CRC_LEN;
        result = kIOReturnSuccess;
    }
    return result;
}

/*! @function setWakeOnMagicPacket
     @abstract Enables or disables the wake on Magic Packet support.
     @discussion Called by enablePacketFilter() or disablePacketFilter()
     when there is a change in the activation state of the Wake-on-LAN
     filter identified by kIOEthernetWakeOnMagicPacket. This method is
     called from the workloop context.
     @param active True to enable support for system wake on reception
     of a Magic Packet, false to disable it.
     @result Returns kIOReturnUnsupported. If overridden, drivers must return
     kIOReturnSuccess on success, or an error return code otherwise.
*/

IOReturn RTL8100::setWakeOnMagicPacket(bool active)
{
    IOReturn result = kIOReturnUnsupported;
    
    DebugLog("setWakeOnMagicPacket() ===>\n");
    
    if (wolCapable) {
        linuxData.wol_enabled = active ? WOL_ENABLED : WOL_DISABLED;
        wolActive = active;
        result = kIOReturnSuccess;
    }
    
    DebugLog("setWakeOnMagicPacket() <===\n");
    
    return result;
}

/*! @function getPacketFilters
     @abstract Gets the set of packet filters supported by the Ethernet
     controller in the given filter group.
     @discussion The default implementation of the abstract method inherited
     from IONetworkController. When the filter group specified is
     gIONetworkFilterGroup, then this method will return a value formed by
     a bitwise OR of kIOPacketFilterUnicast, kIOPacketFilterBroadcast,
     kIOPacketFilterMulticast, kIOPacketFilterPromiscuous. Otherwise, the
     return value will be set to zero (0). Subclasses must override this
     method if their filtering capability differs from what is reported by
     this default implementation. This method is called from the workloop
     context, and the result is published to the I/O Kit Registry.
     @param group The name of the filter group.
     @param filters Pointer to the mask of supported filters returned by
     this method.
     @result Returns kIOReturnSuccess. Drivers that override this
     method must return kIOReturnSuccess to indicate success, or an error
     return code otherwise.
*/

IOReturn RTL8100::getPacketFilters(const OSSymbol *group, UInt32 *filters) const
{
    IOReturn result = kIOReturnSuccess;
    
    DebugLog("getPacketFilters() ===>\n");
    
    if ((group == gIOEthernetWakeOnLANFilterGroup) && wolCapable) {
        *filters = kIOEthernetWakeOnMagicPacket;
        DebugLog("Ethernet [RealtekRTL8100]: kIOEthernetWakeOnMagicPacket added to filters.\n");
    } else {
        result = super::getPacketFilters(group, filters);
    }
    
    DebugLog("getPacketFilters() <===\n");
    
    return result;
}

/*! @function getFeatures
     @abstract Reports generic features supported by the controller and/or
     the driver.
     @result Returns a bit mask of all supported features. */

UInt32 RTL8100::getFeatures() const
{
    UInt32 features = (kIONetworkFeatureMultiPages | kIONetworkFeatureHardwareVlan);
    
    DebugLog("getFeatures() ===>\n");
    
    if (enableTSO4)
        features |= kIONetworkFeatureTSOIPv4;
    
    if (enableTSO6 && revision2)
        features |= kIONetworkFeatureTSOIPv6;
    
    DebugLog("getFeatures() <===\n");
    
    return features;
}

/*! @function selectMedium
     @abstract A client request to change the medium selection.
     @discussion This method is called when a client issues a command
     for the controller to change its current medium selection.
     The implementation must call setSelectedMedium() after the change
     has occurred. This method call is synchronized by the workloop's
     gate.
     @param medium An entry from the published medium dictionary that
     represents the selection chosen by the client.
     @result Returns kIOReturnUnsupported. Drivers may override this method and
     return kIOReturnSuccess if the selection was successful,
     or an error code otherwise.
*/

IOReturn RTL8100::selectMedium(const IONetworkMedium *medium)
{
    IOReturn result = kIOReturnSuccess;
    
    DebugLog("selectMedium() ===>\n");
    
    if (medium) {
        flowCtl = kFlowControlOff;
        eeeAdv = 0;
        
        switch (medium->getIndex()) {
            case MEDIUM_INDEX_AUTO:
                autoneg = AUTONEG_ENABLE;
                speed = SPEED_1000;
                duplex = DUPLEX_FULL;
                eeeAdv = eeeCap;
                break;
                
            case MEDIUM_INDEX_10HD:
                autoneg = AUTONEG_DISABLE;
                speed = SPEED_10;
                duplex = DUPLEX_HALF;
                break;
                
            case MEDIUM_INDEX_10FD:
                autoneg = AUTONEG_ENABLE;
                speed = SPEED_10;
                duplex = DUPLEX_FULL;
                break;
                
            case MEDIUM_INDEX_100HD:
                autoneg = AUTONEG_DISABLE;
                speed = SPEED_100;
                duplex = DUPLEX_HALF;
                break;
                
            case MEDIUM_INDEX_100FD:
                autoneg = AUTONEG_ENABLE;
                speed = SPEED_100;
                duplex = DUPLEX_FULL;
                break;
                
            case MEDIUM_INDEX_100FDFC:
                autoneg = AUTONEG_ENABLE;
                speed = SPEED_100;
                duplex = DUPLEX_FULL;
                flowCtl = kFlowControlOn;
                break;
                
            case MEDIUM_INDEX_100FDEEE:
                autoneg = AUTONEG_ENABLE;
                speed = SPEED_100;
                duplex = DUPLEX_FULL;
                eeeAdv = eeeCap;
                break;
                
            case MEDIUM_INDEX_100FDFCEEE:
                autoneg = AUTONEG_ENABLE;
                speed = SPEED_100;
                duplex = DUPLEX_FULL;
                flowCtl = kFlowControlOn;
                eeeAdv = eeeCap;
                break;
        }
        setPhyMedium();
        setCurrentMedium(medium);
    }
    
    DebugLog("selectMedium() <===\n");
    
done:
    return result;
}

#pragma mark --- data structure initialization methods ---

/*
 * Get configuration parameters from the driver's Info.plist.
 */
void RTL8100::getParams()
{
    OSNumber *intrMit;
    OSBoolean *poll;
    OSBoolean *tso4;
    OSBoolean *tso6;
    OSBoolean *csoV6;
    OSBoolean *noASPM;
    OSString *versionString;
    
    noASPM = OSDynamicCast(OSBoolean, getProperty(kDisableASPMName));
    disableASPM = (noASPM) ? noASPM->getValue() : false;
    
    DebugLog("Ethernet [RealtekRTL8100]: PCIe ASPM support %s.\n", disableASPM ? offName : onName);
    
    enableEEE = OSDynamicCast(OSBoolean, getProperty(kEnableEeeName));
    
    IOLog("Ethernet [RealtekRTL8100]: EEE support %s.\n", enableEEE ? onName : offName);
    
    poll = OSDynamicCast(OSBoolean, getProperty(kEnableRxPollName));
    rxPoll = (poll) ? poll->getValue() : false;
    
    IOLog("Ethernet [RealtekRTL8100]: RxPoll support %s.\n", rxPoll ? onName : offName);
    
    tso4 = OSDynamicCast(OSBoolean, getProperty(kEnableTSO4Name));
    enableTSO4 = (tso4) ? tso4->getValue() : false;
    
    IOLog("Ethernet [RealtekRTL8100]: TCP/IPv4 segmentation offload %s.\n", enableTSO4 ? onName : offName);
    
    tso6 = OSDynamicCast(OSBoolean, getProperty(kEnableTSO6Name));
    enableTSO6 = (tso6) ? tso6->getValue() : false;
    
    IOLog("Ethernet [RealtekRTL8100]: TCP/IPv6 segmentation offload %s.\n", enableTSO6 ? onName : offName);
    
    csoV6 = OSDynamicCast(OSBoolean, getProperty(kEnableCSO6Name));
    enableCSO6 = (csoV6) ? csoV6->getValue() : false;
    
    IOLog("Ethernet [RealtekRTL8100]: TCP/IPv6 checksum offload %s.\n", enableCSO6 ? onName : offName);
    
    intrMit = OSDynamicCast(OSNumber, getProperty(kIntrMitigateName));
    
    if (intrMit && !rxPoll)
        intrMitigateValue = intrMit->unsigned16BitValue();
    
    versionString = OSDynamicCast(OSString, getProperty(kDriverVersionName));
    
    if (versionString)
        IOLog("Ethernet [RealtekRTL8100]: Version %s using interrupt mitigate value 0x%x. Please don't support tonymacx86.com!\n", versionString->getCStringNoCopy(), intrMitigateValue);
    else
        IOLog("Ethernet [RealtekRTL8100]: Using interrupt mitigate value 0x%x. Please don't support tonymacx86.com!\n", intrMitigateValue);
}

static IOMediumType mediumTypeArray[MEDIUM_INDEX_COUNT] = {
    kIOMediumEthernetAuto,
    (kIOMediumEthernet10BaseT | kIOMediumOptionHalfDuplex),
    (kIOMediumEthernet10BaseT | kIOMediumOptionFullDuplex),
    (kIOMediumEthernet100BaseTX | kIOMediumOptionHalfDuplex),
    (kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex),
    (kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex | kIOMediumOptionFlowControl),
    (kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex | kIOMediumOptionEEE),
    (kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex | kIOMediumOptionFlowControl | kIOMediumOptionEEE),
};

static UInt32 mediumSpeedArray[MEDIUM_INDEX_COUNT] = {
    0,
    10 * MBit,
    10 * MBit,
    100 * MBit,
    100 * MBit,
    100 * MBit,
    100 * MBit,
    100 * MBit
};

/*
 * Creates an OSDictionary with with the chip's supported media.
 */
bool RTL8100::setupMediumDict()
{
	IONetworkMedium *medium;
    UInt32 i, n;
    bool result = false;
    
    n = eeeCap ? MEDIUM_INDEX_COUNT : MEDIUM_INDEX_COUNT - 2;
    mediumDict = OSDictionary::withCapacity(n + 1);
    
    if (mediumDict) {
        for (i = MEDIUM_INDEX_AUTO; i < n; i++) {
            medium = IONetworkMedium::medium(mediumTypeArray[i], mediumSpeedArray[i], 0, i);
            
            if (!medium)
                goto error1;
            
            result = IONetworkMedium::addMedium(mediumDict, medium);
            medium->release();
            
            if (!result)
                goto error1;
            
            mediumTable[i] = medium;
        }
    }
    result = publishMediumDictionary(mediumDict);
    
    if (!result)
        goto error1;
    
done:
    return result;
    
error1:
    IOLog("Ethernet [RealtekRTL8100]: Error creating medium dictionary.\n");
    mediumDict->release();
    
    for (i = MEDIUM_INDEX_AUTO; i < MEDIUM_INDEX_COUNT; i++)
        mediumTable[i] = NULL;
    
    goto done;
}

/*
 * Creates and initializes the interrupt handler and the watchdog timer
 * adding them as event sources to the driver's workloop.
 */
bool RTL8100::initEventSources(IOService *provider)
{
    IOReturn intrResult;
    int msiIndex = -1;
    int intrIndex = 0;
    int intrType = 0;
    bool result = false;
    
    txQueue = reinterpret_cast<IOBasicOutputQueue *>(getOutputQueue());
    
    if (txQueue == NULL) {
        IOLog("Ethernet [RealtekRTL8100]: Failed to get output queue.\n");
        goto done;
    }
    txQueue->retain();
    
    while ((intrResult = pciDevice->getInterruptType(intrIndex, &intrType)) == kIOReturnSuccess) {
        if (intrType & kIOInterruptTypePCIMessaged){
            msiIndex = intrIndex;
            break;
        }
        intrIndex++;
    }
    if (msiIndex != -1) {
        DebugLog("Ethernet [RealtekRTL8100]: MSI interrupt index: %d\n", msiIndex);
        
        if (rxPoll) {
            interruptSource = IOInterruptEventSource::interruptEventSource(this, OSMemberFunctionCast(IOInterruptEventSource::Action, this, &RTL8100::interruptOccurredPoll), provider, msiIndex);
        } else {
            interruptSource = IOInterruptEventSource::interruptEventSource(this, OSMemberFunctionCast(IOInterruptEventSource::Action, this, &RTL8100::interruptOccurred), provider, msiIndex);
        }
    }
    if (!interruptSource) {
        DebugLog("Ethernet [RealtekRTL8100]: Warning: MSI index was not found or MSI interrupt could not be enabled.\n");
        goto error1;
    }
    workLoop->addEventSource(interruptSource);
    
    timerSource = IOTimerEventSource::timerEventSource(this, OSMemberFunctionCast(IOTimerEventSource::Action, this, &RTL8100::timerActionRTL8100));
    
    if (!timerSource) {
        IOLog("Ethernet [RealtekRTL8100]: Failed to create IOTimerEventSource.\n");
        goto error2;
    }
    workLoop->addEventSource(timerSource);
    
    result = true;
    
done:
    return result;
    
error2:
    workLoop->removeEventSource(interruptSource);
    RELEASE(interruptSource);
    
error1:
    IOLog("Ethernet [RealtekRTL8100]: Error initializing event sources.\n");
    txQueue->release();
    txQueue = NULL;
    goto done;
}

/*
 * Creates and initializes IOBufferMemoryDescriptors for
 *  - the transmitter DMA descriptor ring
 *  - the receiver DMA descriptor ring
 *  - the statistics dump buffer
 */
bool RTL8100::setupDMADescriptors()
{
    IOPhysicalSegment rxSegment;
    mbuf_t spareMbuf[kRxNumSpareMbufs];
    mbuf_t m;
    UInt32 i;
    UInt32 opts1;
    bool result = false;
    
    /* Create transmitter descriptor array. */
    txBufDesc = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task, (kIODirectionInOut | kIOMemoryPhysicallyContiguous | kIOMapInhibitCache), kTxDescSize, 0xFFFFFFFFFFFFFF00ULL);
    
    if (!txBufDesc) {
        IOLog("Ethernet [RealtekRTL8100]: Couldn't alloc txBufDesc.\n");
        goto done;
    }
    if (txBufDesc->prepare() != kIOReturnSuccess) {
        IOLog("Ethernet [RealtekRTL8100]: txBufDesc->prepare() failed.\n");
        goto error1;
    }
    txDescArray = (RtlDmaDesc *)txBufDesc->getBytesNoCopy();
    txPhyAddr = OSSwapHostToLittleInt64(txBufDesc->getPhysicalAddress());
    
    /* Initialize txDescArray. */
    bzero(txDescArray, kTxDescSize);
    txDescArray[kTxLastDesc].opts1 = OSSwapHostToLittleInt32(RingEnd);
    
    for (i = 0; i < kNumTxDesc; i++) {
        txMbufArray[i] = NULL;
    }
    txNextDescIndex = txDirtyDescIndex = 0;
    txNumFreeDesc = kNumTxDesc;
    txMbufCursor = IOMbufNaturalMemoryCursor::withSpecification(0x4000, kMaxSegs);
    
    if (!txMbufCursor) {
        IOLog("Ethernet [RealtekRTL8100]: Couldn't create txMbufCursor.\n");
        goto error2;
    }
    
    /* Create receiver descriptor array. */
    rxBufDesc = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task, (kIODirectionInOut | kIOMemoryPhysicallyContiguous | kIOMapInhibitCache), kRxDescSize, 0xFFFFFFFFFFFFFF00ULL);
    
    if (!rxBufDesc) {
        IOLog("Ethernet [RealtekRTL8100]: Couldn't alloc rxBufDesc.\n");
        goto error3;
    }
    
    if (rxBufDesc->prepare() != kIOReturnSuccess) {
        IOLog("Ethernet [RealtekRTL8100]: rxBufDesc->prepare() failed.\n");
        goto error4;
    }
    rxDescArray = (RtlDmaDesc *)rxBufDesc->getBytesNoCopy();
    rxPhyAddr = OSSwapHostToLittleInt64(rxBufDesc->getPhysicalAddress());
    
    /* Initialize rxDescArray. */
    bzero(rxDescArray, kRxDescSize);
    rxDescArray[kRxLastDesc].opts1 = OSSwapHostToLittleInt32(RingEnd);
    
    for (i = 0; i < kNumRxDesc; i++) {
        rxMbufArray[i] = NULL;
    }
    rxNextDescIndex = 0;
    
    rxMbufCursor = IOMbufNaturalMemoryCursor::withSpecification(PAGE_SIZE, 1);
    
    if (!rxMbufCursor) {
        IOLog("Ethernet [RealtekRTL8100]: Couldn't create rxMbufCursor.\n");
        goto error5;
    }
    /* Alloc receive buffers. */
    for (i = 0; i < kNumRxDesc; i++) {
        m = allocatePacket(kRxBufferPktSize);
        
        if (!m) {
            IOLog("Ethernet [RealtekRTL8100]: Couldn't alloc receive buffer.\n");
            goto error6;
        }
        rxMbufArray[i] = m;
        
        if (rxMbufCursor->getPhysicalSegmentsWithCoalesce(m, &rxSegment, 1) != 1) {
            IOLog("Ethernet [RealtekRTL8100]: getPhysicalSegmentsWithCoalesce() for receive buffer failed.\n");
            goto error6;
        }
        opts1 = (UInt32)rxSegment.length;
        opts1 |= (i == kRxLastDesc) ? (RingEnd | DescOwn) : DescOwn;
        rxDescArray[i].opts1 = OSSwapHostToLittleInt32(opts1);
        rxDescArray[i].opts2 = 0;
        rxDescArray[i].addr = OSSwapHostToLittleInt64(rxSegment.location);
    }
    /* Create statistics dump buffer. */
    statBufDesc = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task, (kIODirectionIn | kIOMemoryPhysicallyContiguous | kIOMapInhibitCache), sizeof(RtlStatData), 0xFFFFFFFFFFFFFF00ULL);
    
    if (!statBufDesc) {
        IOLog("Ethernet [RealtekRTL8100]: Couldn't alloc statBufDesc.\n");
        goto error6;
    }
    
    if (statBufDesc->prepare() != kIOReturnSuccess) {
        IOLog("Ethernet [RealtekRTL8100]: statBufDesc->prepare() failed.\n");
        goto error7;
    }
    statData = (RtlStatData *)statBufDesc->getBytesNoCopy();
    statPhyAddr = OSSwapHostToLittleInt64(statBufDesc->getPhysicalAddress());
    
    /* Initialize statData. */
    bzero(statData, sizeof(RtlStatData));
    
    /* Allocate some spare mbufs and free them in order to increase the buffer pool.
     * This seems to avoid the replaceOrCopyPacket() errors under heavy load.
     */
    for (i = 0; i < kRxNumSpareMbufs; i++)
        spareMbuf[i] = allocatePacket(kRxBufferPktSize);
    
    for (i = 0; i < kRxNumSpareMbufs; i++) {
        if (spareMbuf[i])
            freePacket(spareMbuf[i]);
    }
    result = true;
    
done:
    return result;
    
error7:
    statBufDesc->release();
    statBufDesc = NULL;
    
error6:
    for (i = 0; i < kNumRxDesc; i++) {
        if (rxMbufArray[i]) {
            freePacket(rxMbufArray[i]);
            rxMbufArray[i] = NULL;
        }
    }
    RELEASE(rxMbufCursor);
    
error5:
    rxBufDesc->complete();
    
error4:
    rxBufDesc->release();
    rxBufDesc = NULL;
    
error3:
    RELEASE(txMbufCursor);
    
error2:
    txBufDesc->complete();
    
error1:
    txBufDesc->release();
    txBufDesc = NULL;
    goto done;
}

void RTL8100::freeDMADescriptors()
{
    UInt32 i;
    
    if (txBufDesc) {
        txBufDesc->complete();
        txBufDesc->release();
        txBufDesc = NULL;
        txPhyAddr = NULL;
    }
    RELEASE(txMbufCursor);
    
    if (rxBufDesc) {
        rxBufDesc->complete();
        rxBufDesc->release();
        rxBufDesc = NULL;
        rxPhyAddr = NULL;
    }
    RELEASE(rxMbufCursor);
    
    for (i = 0; i < kNumRxDesc; i++) {
        if (rxMbufArray[i]) {
            freePacket(rxMbufArray[i]);
            rxMbufArray[i] = NULL;
        }
    }
    if (statBufDesc) {
        statBufDesc->complete();
        statBufDesc->release();
        statBufDesc = NULL;
        statPhyAddr = NULL;
        statData = NULL;
    }
}

/*
 * Clears the tx descriptor ring freeing all packets in the queue.
 * Called when the link was lost or the interface is disabled.
 */
void RTL8100::txClearDescriptors()
{
    mbuf_t m;
    UInt32 lastIndex = kTxLastDesc;
    UInt32 i;
    
    DebugLog("txClearDescriptors() ===>\n");
    
    if (txNext2FreeMbuf) {
        freePacket(txNext2FreeMbuf);
        txNext2FreeMbuf = NULL;
    }
    for (i = 0; i < kNumTxDesc; i++) {
        txDescArray[i].opts1 = OSSwapHostToLittleInt32((i != lastIndex) ? 0 : RingEnd);
        m = txMbufArray[i];
        
        if (m) {
            freePacket(m);
            txMbufArray[i] = NULL;
        }
    }
    txDirtyDescIndex = txNextDescIndex = 0;
    txNumFreeDesc = kNumTxDesc;
    
    DebugLog("txClearDescriptors() <===\n");
}

#pragma mark --- common interrupt methods ---

void RTL8100::pciErrorInterrupt()
{
    UInt16 cmdReg = pciDevice->configRead16(kIOPCIConfigCommand);
    UInt16 statusReg = pciDevice->configRead16(kIOPCIConfigStatus);
    
    DebugLog("Ethernet [RealtekRTL8100]: PCI error: cmdReg=0x%x, statusReg=0x%x\n", cmdReg, statusReg);
    
    cmdReg |= (kIOPCICommandSERR | kIOPCICommandParityError);
    statusReg &= (kIOPCIStatusParityErrActive | kIOPCIStatusSERRActive | kIOPCIStatusMasterAbortActive | kIOPCIStatusTargetAbortActive | kIOPCIStatusTargetAbortCapable);
    pciDevice->extendedConfigWrite16(kIOPCIConfigCommand, cmdReg);
    pciDevice->extendedConfigWrite16(kIOPCIConfigStatus, statusReg);
    
    /* Reset the NIC in order to resume operation. */
    restartRTL8100();
}

/* 
 * Transmitter interrupt handler
 *
 * Some (all?) of the RTL8100 family members don't handle descriptors properly.
 * They randomly release control of descriptors pointing to certain packets
 * before the request has been completed and reclaim them later.
 *
 * As a workaround we should:
 * - leave returned descriptors untouched until they get reused.
 * - never reuse the descriptor which has been returned last, i.e. leave at
 *   least one of the descriptors in txDescArray unused.
 * - delay freeing packets until the next descriptor has been finished or a
 *   small period of time has passed (as these packets are really small a
 *   few µ secs should be enough).
 */

void RTL8100::txInterrupt()
{
    SInt32 numDirty = kNumTxDesc - txNumFreeDesc;
    UInt32 oldDirtyIndex = txDirtyDescIndex;
    UInt32 descStatus;
    
    while (numDirty-- > 0) {
        descStatus = OSSwapLittleToHostInt32(txDescArray[txDirtyDescIndex].opts1);
        
        if (descStatus & DescOwn)
            break;
        
        /* Now it's time to free the last mbuf as we can be sure it's not in use anymore. */
        if (txNext2FreeMbuf)
            freePacket(txNext2FreeMbuf, kDelayFree);
        
        txNext2FreeMbuf = txMbufArray[txDirtyDescIndex];
        txMbufArray[txDirtyDescIndex] = NULL;
        txDescDoneCount++;
        OSIncrementAtomic(&txNumFreeDesc);
        ++txDirtyDescIndex &= kTxDescMask;
    }
    if (oldDirtyIndex != txDirtyDescIndex) {
        if (txNumFreeDesc > kTxQueueWakeTreshhold)
            netif->signalOutputThread();
        
        WriteReg8(TxPoll, NPQ);
        releaseFreePackets();
    }
    if (!polling)
        etherStats->dot3TxExtraEntry.interrupts++;
}

/*
 * Receiver interrupt handler
 *
 * Also called by pollInputPackets() while polling for received packets is acitve.
 */
UInt32 RTL8100::rxInterrupt(IONetworkInterface *interface, uint32_t maxCount, IOMbufQueue *pollQueue, void *context)
{
    IOPhysicalSegment rxSegment;
    RtlDmaDesc *desc = &rxDescArray[rxNextDescIndex];
    mbuf_t bufPkt, newPkt;
    UInt64 addr;
    UInt32 opts1, opts2;
    UInt32 descStatus1, descStatus2;
    UInt32 pktSize;
    UInt32 goodPkts = 0;
    UInt16 vlanTag;
    bool replaced;
    
    while (!((descStatus1 = OSSwapLittleToHostInt32(desc->opts1)) & DescOwn) && (goodPkts < maxCount)) {
        opts1 = (rxNextDescIndex == kRxLastDesc) ? (RingEnd | DescOwn) : DescOwn;
        opts2 = 0;
        addr = 0;
        
        /* As we don't support jumbo frames we consider fragmented packets as errors. */
        if ((descStatus1 & (FirstFrag|LastFrag)) != (FirstFrag|LastFrag)) {
            DebugLog("Ethernet [RealtekRTL8100]: Fragmented packet.\n");
            etherStats->dot3StatsEntry.frameTooLongs++;
            opts1 |= kRxBufferPktSize;
            goto nextDesc;
        }
        
        descStatus2 = OSSwapLittleToHostInt32(desc->opts2);
        pktSize = (descStatus1 & 0x1fff) - kIOEthernetCRCSize;
        bufPkt = rxMbufArray[rxNextDescIndex];
        vlanTag = (descStatus2 & RxVlanTag) ? OSSwapInt16(descStatus2 & 0xffff) : 0;
        //DebugLog("rxInterrupt(): descStatus1=0x%x, descStatus2=0x%x, pktSize=%u\n", descStatus1, descStatus2, pktSize);
        
        newPkt = replaceOrCopyPacket(&bufPkt, pktSize, &replaced);
        
        if (!newPkt) {
            /* Allocation of a new packet failed so that we must leave the original packet in place. */
            DebugLog("Ethernet [RealtekRTL8100]: replaceOrCopyPacket() failed.\n");
            etherStats->dot3RxExtraEntry.resourceErrors++;
            opts1 |= kRxBufferPktSize;
            goto nextDesc;
        }
        
        /* If the packet was replaced we have to update the descriptor's buffer address. */
        if (replaced) {
            if (rxMbufCursor->getPhysicalSegments(bufPkt, &rxSegment, 1) != 1) {
                DebugLog("Ethernet [RealtekRTL8100]: getPhysicalSegments() failed.\n");
                etherStats->dot3RxExtraEntry.resourceErrors++;
                freePacket(bufPkt);
                opts1 |= kRxBufferPktSize;
                goto nextDesc;
            }
            opts1 |= ((UInt32)rxSegment.length & 0x0000ffff);
            addr = rxSegment.location;
            rxMbufArray[rxNextDescIndex] = bufPkt;
        } else {
            opts1 |= kRxBufferPktSize;
        }
        getChecksumResult(newPkt, descStatus1, descStatus2);
        
        /* Also get the VLAN tag if there is any. */
        if (vlanTag)
            setVlanTag(newPkt, vlanTag);
        
        mbuf_pkthdr_setlen(newPkt, pktSize);
        mbuf_setlen(newPkt, pktSize);
        interface->enqueueInputPacket(newPkt, pollQueue);
        goodPkts++;
        
        /* Finally update the descriptor and get the next one to examine. */
    nextDesc:
        if (addr)
            desc->addr = OSSwapHostToLittleInt64(addr);
        
        desc->opts2 = OSSwapHostToLittleInt32(opts2);
        desc->opts1 = OSSwapHostToLittleInt32(opts1);
        
        ++rxNextDescIndex &= kRxDescMask;
        desc = &rxDescArray[rxNextDescIndex];
    }
    return goodPkts;
}

/*
 * Interrupt service routine with support for polled receive mode.
 */
void RTL8100::interruptOccurredPoll(OSObject *client, IOInterruptEventSource *src, int count)
{
    UInt32 packets;
    
    UInt16 status;
    
    WriteReg16(IntrMask, 0x0000);
    status = ReadReg16(IntrStatus);
    
    /* hotplug/major error/no more work/shared irq */
    if ((status == 0xFFFF) || !status)
        goto done;
    
    if (status & SYSErr) {
        pciErrorInterrupt();
        goto done;
    }
    if (!polling) {
        /* Rx interrupt */
        if (status & (RxOK | RxDescUnavail | RxFIFOOver)) {
            packets = rxInterrupt(netif, kNumRxDesc, NULL, NULL);
            
            if (packets)
                netif->flushInputQueue();
        }
        /* Tx interrupt */
        if (status & (TxOK | TxErr | TxDescUnavail))
            txInterrupt();
    }
    
done:
    WriteReg16(IntrStatus, status);
    WriteReg16(IntrMask, intrMask);
}

/*
 * Interrupt service routine without support for polled receive mode.
 */
void RTL8100::interruptOccurred(OSObject *client, IOInterruptEventSource *src, int count)
{
    UInt32 packets;
    UInt16 status;
    
	WriteReg16(IntrMask, 0x0000);
    status = ReadReg16(IntrStatus);
    
    /* hotplug/major error/no more work/shared irq */
    if ((status == 0xFFFF) || !status)
        goto done;
        
    if (status & SYSErr)
        pciErrorInterrupt();
    
    /* Rx interrupt */
    if (status & (RxOK | RxDescUnavail | RxFIFOOver)) {
        packets = rxInterrupt(netif, kNumRxDesc, NULL, NULL);
    
        if (packets)
            netif->flushInputQueue();
    }

    /* Tx interrupt */
    if (status & (TxOK | TxErr | TxDescUnavail))
        txInterrupt();
        
    /* Check if a statistics dump has been completed. */
    if (needsUpdate && !(ReadReg32(CounterAddrLow) & CounterDump))
        updateStatitics();
    
done:
    WriteReg16(IntrStatus, status);
	WriteReg16(IntrMask, intrMask);
}

/*
 * Checks the transmitter ring for deadlocks. Called by the watchdog timer periodically.
 * 
 * The algorithm works as follows:
 *  - Are there unprocessed packets in the tx DMA descriptor ring?
 *  - Hasn't there been any progress in tx descriptor processing by hardware?
 *
 * If both conditions are consecutively true for kTxCheckTreshhold seconds, the driver will
 * output a warning that interrrupt requests may have been lost. The tx interrupt handler
 * will be called in order to check the descriptor ring for processed packets. Provided these
 * messages don't fill up the kernel logs, there is no need to worry about.
 *
 * If both conditions are consecutively true for kTxDeadlockTreshhold seconds, a tranmitter
 * hang is assumed and the driver will reset the chip and resume normal operation.
 */
bool RTL8100::checkForDeadlock()
{
    bool deadlock = false;
    
    if ((txDescDoneCount == txDescDoneLast) && (txNumFreeDesc < kNumTxDesc)) {
        if (++deadlockWarn == kTxCheckTreshhold) {
            /* Some members of the RTL8100 family seem to be prone to lose transmitter rinterrupts.
             * In order to avoid false positives when trying to detect transmitter deadlocks, check
             * the transmitter ring once for completed descriptors before we assume a deadlock.
             */
            IOLog("Ethernet [RealtekRTL8100]: Tx timeout. Lost interrupt?\n");
            etherStats->dot3TxExtraEntry.timeouts++;
            txInterrupt();
        } else if (deadlockWarn >= kTxDeadlockTreshhold) {
#ifdef DEBUG
            UInt32 i, index;
            
            for (i = 0; i < 10; i++) {
                index = ((txDirtyDescIndex - 1 + i) & kTxDescMask);
                IOLog("Ethernet [RealtekRTL8100]: desc[%u]: opts1=0x%x, opts2=0x%x, addr=0x%llx.\n", index, txDescArray[index].opts1, txDescArray[index].opts2, txDescArray[index].addr);
            }
#endif
            IOLog("Ethernet [RealtekRTL8100]: Tx stalled? Resetting chipset. ISR=0x%x, IMR=0x%x.\n", ReadReg16(IntrStatus), ReadReg16(IntrMask));
            etherStats->dot3TxExtraEntry.resets++;
            restartRTL8100();
            deadlock = true;
        }
    } else {
        deadlockWarn = 0;
    }
    return deadlock;
}

#pragma mark --- rx poll methods ---

/*! @function setInputPacketPollingEnable
 @abstract Informs the driver when input polling is enabled or disabled.
     @discussion A driver that supports input polling should override this
     method to handle the transition from the default push-model, where a
     hardware interrupt causes the driver to push input packets to the network
     stack, to the pull-model where a poller thread will periodically poll the
     driver for input packets. When polling is enabled, receive interrupt should
     be masked OFF. This method is always called on driver's work loop context.
     @param interface The interface that has enabled or disabled input polling.
     @param enabled <code>true</code> if input polling is enabled,
     <code>false</code> if input polling is disabled.
     @result Driver should return <code>kIOReturnSuccess</code> if the transition
     to/from polled-mode was successful, or an error code otherwise.
 */
IOReturn RTL8100::setInputPacketPollingEnable(IONetworkInterface *interface, bool enabled)
{
    //DebugLog("setInputPacketPollingEnable() ===>\n");
    
    if (enabled) {
        intrMask = intrMaskPoll;
        polling = true;
    } else {
        intrMask = intrMaskRxTx;
        polling = false;
    }
    if(isEnabled)
        WriteReg16(IntrMask, intrMask);
    
    //DebugLog("input polling %s.\n", enabled ? "enabled" : "disabled");
    
    //DebugLog("setInputPacketPollingEnable() <===\n");
    
    return kIOReturnSuccess;
}

/*! @function pollInputPackets
     @abstract Called by the interface poller thread to retrieve input packets
     from the driver.
     @discussion A driver that supports input polling must override this method
     and pass any input packets to the network interface. For each input packet,
     driver must call <code>IONetworkInterface::enqueueInputPacket</code> and
     pass the <code>pollQueue</code> argument to add the packet to the polling
     queue. This should continue until the maximum packet count is reached, or
     the driver runs out of input packets. The poller can be configured to call
     this method on the driver's work loop context.
     @param interface The interface that is polling packets from the driver.
     @param maxCount The maximum number of packets that the poller can accept.
     @param pollQueue The polling queue that should be passed to
     <code>IONetworkInterface::enqueueInputPacket</code>. Do not cache or use
     this pointer after the method returns.
     @param context The family will always pass zero. This can be used by the
     driver for calls originating from the driver. E.g. A driver may choose to
     unify polled-mode and interrupt-mode input packet handling and call this
     method from its receive interrupt handler and pass a non-zero context to
     distinguish the calling context.
 */
void RTL8100::pollInputPackets(IONetworkInterface *interface, uint32_t maxCount, IOMbufQueue *pollQueue, void *context )
{
    //DebugLog("pollInputPackets() ===>\n");
    
    rxInterrupt(interface, maxCount, pollQueue, context);
    
    /* Finally cleanup the transmitter ring. */
    txInterrupt();
    
    //DebugLog("pollInputPackets() <===\n");
}

#pragma mark --- hardware specific methods ---

/*
 * Get command bits for TCP Segmentation Offload operations using TCP/IPv4.
 */
void RTL8100::getTso4Command(UInt32 *cmd1, UInt32 *cmd2, UInt32 mssValue, mbuf_tso_request_flags_t tsoFlags)
{
    if (revision2) {
        *cmd1 = (GiantSendv4 | (kMinL4HdrOffsetV4 << GSendL4OffShift));
        *cmd2 = ((mssValue & MSSMask) << MSSShift_C);
    } else {
        *cmd1 = (LargeSend |((mssValue & MSSMask) << MSSShift));
    }
}

/*
 * Get command bits for TCP Segmentation Offload operations using TCP/IPv6.
 */
void RTL8100::getTso6Command(UInt32 *cmd1, UInt32 *cmd2, UInt32 mssValue, mbuf_tso_request_flags_t tsoFlags)
{
    *cmd1 = (GiantSendv6 | (kMinL4HdrOffsetV6 << GSendL4OffShift));
    *cmd2 = ((mssValue & MSSMask) << MSSShift_C);
}

/*
 * Get command bits for TCP/UDP/IPv4 checksum offload operations.
 */
void RTL8100::getChecksumCommand(UInt32 *cmd1, UInt32 *cmd2, mbuf_csum_request_flags_t checksums)
{
    if (revision2) {
        if (checksums & kChecksumTCP)
            *cmd2 = (TxIPCS_C | TxTCPCS_C);
        else if (checksums & kChecksumUDP)
            *cmd2 = (TxIPCS_C | TxUDPCS_C);
        else if (checksums & kChecksumIP)
            *cmd2 = TxIPCS_C;
        else if (checksums & kChecksumTCPIPv6)
            *cmd2 = (TxTCPCS_C | TxIPV6F_C | ((kMinL4HdrOffsetV6 & L4OffMask) << MSSShift_C));
        else if (checksums & kChecksumUDPIPv6)
            *cmd2 = (TxUDPCS_C | TxIPV6F_C | ((kMinL4HdrOffsetV6 & L4OffMask) << MSSShift_C));
    } else {
        /* Setup the checksum command bits. */
        if (checksums & kChecksumTCP)
            *cmd1 = (TxIPCS | TxTCPCS);
        else if (checksums & kChecksumUDP)
            *cmd1 = (TxIPCS | TxUDPCS);
        else if (checksums & kChecksumIP)
            *cmd1 = TxIPCS;
    }
}

/*
 * Get result of TCP/UDP/IPv4 checksum validation done in hardware.
 * In case hardware checksum validation failed, we tread the packet as 
 * uncheck and let the network stack perform ckecksum validation in
 * software as hardware checksum validation may produce false negatives
 * sometimes.
 */

#ifdef DEBUG

void RTL8100::getChecksumResult(mbuf_t m, UInt32 status1, UInt32 status2)
{
    UInt32 resultMask = 0;
    UInt32 validMask = 0;
    UInt32 pktType = (status1 & RxProtoMask);
    
    /* Get the result of the checksum calculation and store it in the packet. */
    if (revision2) {
        if (pktType == RxTCPT) {
            /* TCP packet */
            if (status2 & RxV4F) {
                resultMask = (kChecksumTCP | kChecksumIP);
                validMask = (status1 & RxTCPF) ? 0 : (kChecksumTCP | kChecksumIP);
            } else if (status2 & RxV6F) {
                resultMask = kChecksumTCPIPv6;
                validMask = (status1 & RxTCPF) ? 0 : kChecksumTCPIPv6;
            }
        } else if (pktType == RxUDPT) {
            /* UDP packet */
            if (status2 & RxV4F) {
                resultMask = (kChecksumUDP | kChecksumIP);
                validMask = (status1 & RxUDPF) ? 0 : (kChecksumUDP | kChecksumIP);
            } else if (status2 & RxV6F) {
                resultMask = kChecksumUDPIPv6;
                validMask = (status1 & RxUDPF) ? 0 : kChecksumUDPIPv6;
            }
        } else if ((pktType == 0) && (status2 & RxV4F)) {
            /* IP packet */
            resultMask = kChecksumIP;
            validMask = (status1 & RxIPF) ? 0 : kChecksumIP;
        }
    } else {
        if (pktType == RxProtoTCP) {
            /* TCP packet */
            resultMask = (kChecksumTCP | kChecksumIP);
            validMask = (status1 & RxTCPF) ? 0 : (kChecksumTCP | kChecksumIP);
        } else if (pktType == RxProtoUDP) {
            /* UDP packet */
            resultMask = (kChecksumUDP | kChecksumIP);
            validMask = (status1 & RxUDPF) ? 0 : (kChecksumUDP | kChecksumIP);
        } else if (pktType == RxProtoIP) {
            /* IP packet */
            resultMask = kChecksumIP;
            validMask = (status1 & RxIPF) ? 0 : kChecksumIP;
        }
    }
    if (validMask != resultMask)
        IOLog("Ethernet [RealtekRTL8100]: checksums applied: 0x%x, checksums valid: 0x%x\n", resultMask, validMask);
    
    if (validMask)
        setChecksumResult(m, kChecksumFamilyTCPIP, resultMask, validMask);
}

#else

void RTL8100::getChecksumResult(mbuf_t m, UInt32 status1, UInt32 status2)
{
    UInt32 resultMask = 0;
    UInt32 pktType = (status1 & RxProtoMask);
    
    if (revision2) {
        /* Get the result of the checksum calculation and store it in the packet. */
        if (pktType == RxTCPT) {
            /* TCP packet */
            if (status2 & RxV4F)
                resultMask = (status1 & RxTCPF) ? 0 : (kChecksumTCP | kChecksumIP);
            else if (status2 & RxV6F)
                resultMask = (status1 & RxTCPF) ? 0 : kChecksumTCPIPv6;
        } else if (pktType == RxUDPT) {
            /* UDP packet */
            if (status2 & RxV4F)
                resultMask = (status1 & RxUDPF) ? 0 : (kChecksumUDP | kChecksumIP);
            else if (status2 & RxV6F)
                resultMask = (status1 & RxUDPF) ? 0 : kChecksumUDPIPv6;
        } else if ((pktType == 0) && (status2 & RxV4F)) {
            /* IP packet */
            resultMask = (status1 & RxIPF) ? 0 : kChecksumIP;
        }
    } else {
        if (pktType == RxProtoTCP)
            resultMask = (status1 & RxTCPF) ? 0 : (kChecksumTCP | kChecksumIP);  /* TCP packet */
        else if (pktType == RxProtoUDP)
            resultMask = (status1 & RxUDPF) ? 0 : (kChecksumUDP | kChecksumIP);  /* UDP packet */
        else if (pktType == RxProtoIP)
            resultMask = (status1 & RxIPF) ? 0 : kChecksumIP;                    /* IP packet */
    }
    if (resultMask)
        setChecksumResult(m, kChecksumFamilyTCPIP, resultMask, resultMask);
}

#endif

static const char *speed100MName = "100-Megabit";
static const char *speed10MName = "10-Megabit";
static const char *duplexFullName = "Full-duplex";
static const char *duplexHalfName = "Half-duplex";
static const char *offFlowName = "No flow-control";
static const char *onFlowName = "flow-control";

static const char* eeeNames[kEEETypeCount] = {
    "",
    ", energy-efficient-ethernet"
};

/*
 * Called when a link has been established to determine the media parameters
 * (speed, duplex, flow control and EEE status), report the media to the
 * network stack and start packet transmission.
 */
void RTL8100::setLinkUp(UInt8 linkState)
{
    UInt64 mediumSpeed;
    UInt32 mediumIndex = MEDIUM_INDEX_AUTO;
    const char *speedName;
    const char *duplexName;
    const char *flowName;
    const char *eeeName;
    UInt16 newIntrMitigate = 0x5f51;
    UInt16 eee = 0;
    
    eeeName = eeeNames[kEEETypeNo];
    
    /* Get EEE mode. */
    if (eeeCap) {
        mdio_write(&linuxData, 0x0D, 0x0007);
        mdio_write(&linuxData, 0x0E, 0x003D);
        mdio_write(&linuxData, 0x0D, 0x4007);
        eee = (mdio_read(&linuxData, 0x0E) & eeeAdv);
        DebugLog("Ethernet [RealtekRTL8100]: EEE Advertise: 0x%x, Link Partner Ability: 0x%x\n", eeeAdv, eee);
    }
    /* Get link speed, duplex and flow-control mode. */
    if (linkState &	(TxFlowCtrl | RxFlowCtrl)) {
        flowName = onFlowName;
        flowCtl = kFlowControlOn;
    } else {
        flowName = offFlowName;
        flowCtl = kFlowControlOff;
    }
    if (linkState & _100bps) {
        mediumSpeed = kSpeed100MBit;
        speed = SPEED_100;
        speedName = speed100MName;
        
        if (linkState & FullDup) {
            duplexName = duplexFullName;
            
            if (flowCtl == kFlowControlOn) {
                if (eee & kEEEMode100) {
                    mediumIndex =  MEDIUM_INDEX_100FDFCEEE;
                    eeeName = eeeNames[kEEETypeYes];
                } else {
                    mediumIndex = MEDIUM_INDEX_100FDFC;
                }
            } else {
                if (eee & kEEEMode100) {
                    mediumIndex =  MEDIUM_INDEX_100FDEEE;
                    eeeName = eeeNames[kEEETypeYes];
                } else {
                    mediumIndex = MEDIUM_INDEX_100FD;
                }
            }
        } else {
            mediumIndex = MEDIUM_INDEX_100HD;
            duplexName = duplexHalfName;
        }
    } else {
        mediumSpeed = kSpeed10MBit;
        speed = SPEED_10;
        speedName = speed10MName;
        
        if (linkState & FullDup) {
            mediumIndex = MEDIUM_INDEX_10FD;
            duplexName = duplexFullName;
        } else {
            mediumIndex = MEDIUM_INDEX_10HD;
            duplexName = duplexHalfName;
        }
    }
    startRTL8100(newIntrMitigate, false);
    linkUp = true;
    setLinkStatus(kIONetworkLinkValid | kIONetworkLinkActive, mediumTable[mediumIndex], mediumSpeed, NULL);
    
    /* Start output thread, statistics update and watchdog. */
    if (rxPoll) {
        /* Update poll params according to link speed. */
        bzero(&pollParams, sizeof(IONetworkPacketPollingParameters));
        
        if (speed == SPEED_10) {
            pollParams.lowThresholdPackets = 2;
            pollParams.highThresholdPackets = 8;
            pollParams.lowThresholdBytes = 0x400;
            pollParams.highThresholdBytes = 0x1800;
            pollParams.pollIntervalTime = 1000000;  /* 1ms */
        } else {
            pollParams.lowThresholdPackets = 10;
            pollParams.highThresholdPackets = 40;
            pollParams.lowThresholdBytes = 0x1000;
            pollParams.highThresholdBytes = 0x10000;
            pollParams.pollIntervalTime = 1000000;  /* 1ms */
        }
        netif->setPacketPollingParameters(&pollParams, 0);
        DebugLog("Ethernet [RealtekRTL8100]: pollIntervalTime: %lluus\n", (pollParams.pollIntervalTime / 1000));
    }
    netif->startOutputThread();
    
    IOLog("Ethernet [RealtekRTL8100]: Link up on en%u, %s, %s, %s%s\n", netif->getUnitNumber(), speedName, duplexName, flowName, eeeName);
}

/*
 * Called when the link was lost. Stops packet transmission,
 * clears the tx descriptor ring and reports to the network
 * stack.
 */
void RTL8100::setLinkDown()
{
    deadlockWarn = 0;
    needsUpdate = false;
    
    /* Stop output thread and flush output queue. */
    netif->stopOutputThread();
    netif->flushOutputQueue();
    
    /* Update link status. */
    linkUp = false;
    setLinkStatus(kIONetworkLinkValid);
    
    rtl8101_nic_reset(&linuxData);
    
    /* Cleanup descriptor ring. */
    txClearDescriptors();
    
    setPhyMedium();

    IOLog("Ethernet [RealtekRTL8100]: Link down on en%u\n", netif->getUnitNumber());
}

/*
 * Configure the PHY for the selected media and/or auto-configuration. Also
 * enable/disable flow control and EEE advertisement.
 *
 * Most of the code is taken from the Linux driver's rtl8101_set_speed_xmii()
 * routine.
 */
void RTL8100::setPhyMedium()
{
    struct rtl8101_private *tp = &linuxData;
    int auto_nego = 0;
    int bmcr_true_force = 0;
    
    if (tp->mcfg == CFG_METHOD_18 || tp->mcfg == CFG_METHOD_19) {
        //Disable Giga Lite
        spin_lock_irqsave(&tp->phy_lock, flags);
        mdio_write(tp, 0x1F, 0x0A42);
        ClearEthPhyBit(tp, 0x14, BIT_9);
        mdio_write(tp, 0x1F, 0x0A40);
        mdio_write(tp, 0x1F, 0x0000);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
    }
    
    if ((speed != SPEED_100) &&
        (speed != SPEED_10)) {
        speed = SPEED_100;
        duplex = DUPLEX_FULL;
    }
    
    auto_nego = mdio_read(tp, MII_ADVERTISE);
    
    auto_nego &= ~(ADVERTISE_10HALF | ADVERTISE_10FULL | ADVERTISE_100HALF | ADVERTISE_100FULL | ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM);
    
    if (autoneg == AUTONEG_ENABLE) {
        /*n-way force*/
        if ((speed == SPEED_10) && (duplex == DUPLEX_HALF)) {
            auto_nego |= ADVERTISE_10HALF;
        } else if ((speed == SPEED_10) && (duplex == DUPLEX_FULL)) {
            auto_nego |= ADVERTISE_10HALF |
            ADVERTISE_10FULL;
        } else if ((speed == SPEED_100) && (duplex == DUPLEX_HALF)) {
            auto_nego |= ADVERTISE_100HALF |
            ADVERTISE_10HALF |
            ADVERTISE_10FULL;
        } else if ((speed == SPEED_100) && (duplex == DUPLEX_FULL)) {
            auto_nego |= ADVERTISE_100HALF |
            ADVERTISE_100FULL |
            ADVERTISE_10HALF |
            ADVERTISE_10FULL;
        }
        
        /* Set flow control support. */
        if (flowCtl)
            auto_nego |= ADVERTISE_PAUSE_CAP|ADVERTISE_PAUSE_ASYM;
        
        if ((tp->mcfg == CFG_METHOD_4) || (tp->mcfg == CFG_METHOD_5) ||
            (tp->mcfg == CFG_METHOD_6) || (tp->mcfg == CFG_METHOD_7) ||
            (tp->mcfg == CFG_METHOD_8) || (tp->mcfg == CFG_METHOD_9)) {
            auto_nego &= ~(ADVERTISE_PAUSE_CAP|ADVERTISE_PAUSE_ASYM);
        }
        
        tp->phy_auto_nego_reg = auto_nego;
        
        if ((tp->mcfg == CFG_METHOD_4) ||
            (tp->mcfg == CFG_METHOD_5)) {
            mdio_write(tp, 0x1f, 0x0000);
            mdio_write(tp, MII_BMCR, BMCR_RESET);
            udelay(100);
            rtl8101_hw_phy_config(tp);
        } else if (((tp->mcfg == CFG_METHOD_1) ||
                    (tp->mcfg == CFG_METHOD_2) ||
                    (tp->mcfg == CFG_METHOD_3)) &&
                   (speed == SPEED_10)) {
            mdio_write(tp, 0x1f, 0x0000);
            mdio_write(tp, MII_BMCR, BMCR_RESET);
            rtl8101_hw_phy_config(tp);
        }
        
        mdio_write(tp, 0x1f, 0x0000);
        mdio_write(tp, MII_ADVERTISE, auto_nego);
        
        /* Setup EEE advertisemnet. */
        if (tp->HwHasWrRamCodeToMicroP == TRUE) {
            if (eeeAdv)
                enableEEESupport();
            else
                disableEEESupport();
        }
        
        if (tp->mcfg == CFG_METHOD_10)
            mdio_write(tp, MII_BMCR, BMCR_RESET | BMCR_ANENABLE | BMCR_ANRESTART);
        else
            mdio_write(tp, MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);

        mdelay(20);
    } else {
        /*true force*/
        
        if ((speed == SPEED_10) && (duplex == DUPLEX_HALF)) {
            bmcr_true_force = BMCR_SPEED10;
        } else if ((speed == SPEED_10) && (duplex == DUPLEX_FULL)) {
            bmcr_true_force = BMCR_SPEED10 |
            BMCR_FULLDPLX;
        } else if ((speed == SPEED_100) && (duplex == DUPLEX_HALF)) {
            bmcr_true_force = BMCR_SPEED100;
        } else if ((speed == SPEED_100) && (duplex == DUPLEX_FULL)) {
            bmcr_true_force = BMCR_SPEED100 |
            BMCR_FULLDPLX;
        }
        
        mdio_write(tp, 0x1f, 0x0000);
        mdio_write(tp, MII_BMCR, bmcr_true_force);
    }
    
    tp->autoneg = autoneg;
    tp->speed = speed;
    tp->duplex = duplex;
}

/*
 * Slightly modified version of the Linux driver's rtl8101_enable_EEE()
 * routine.
 */
void RTL8100::enableEEESupport()
{
    struct rtl8101_private *tp = &linuxData;
    UInt16 data;
    UInt16 PhyRegValue;
    UInt32 WaitCnt;
    
    switch (tp->mcfg) {
        case CFG_METHOD_10:
            mdio_write(tp, 0x1F, 0x0007);
            mdio_write(tp, 0x1E, 0x0020);
            data = mdio_read(tp, 0x15) | 0x0100;
            mdio_write(tp, 0x15, data);
            mdio_write(tp, 0x1F, 0x0006);
            mdio_write(tp, 0x00, 0x5A30);
            mdio_write(tp, 0x1F, 0x0000);
            mdio_write(tp, 0x0D, 0x0007);
            mdio_write(tp, 0x0E, 0x003C);
            mdio_write(tp, 0x0D, 0x4007);
            mdio_write(tp, 0x0E, 0x0006);
            mdio_write(tp, 0x0D, 0x0000);
            
            if ((ReadReg8(Config4)&0x40) && (ReadReg8(0x6D) & BIT_7)) {
                mdio_write(tp, 0x1F, 0x0005);
                mdio_write(tp, 0x05, 0x8AC8);
                mdio_write(tp, 0x06, ReadReg16(CustomLED));
                mdio_write(tp, 0x05, 0x8B82);
                data = mdio_read(tp, 0x06) | 0x0010;
                mdio_write(tp, 0x05, 0x8B82);
                mdio_write(tp, 0x06, data);
                mdio_write(tp, 0x1F, 0x0000);
            }
            break;
            
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
            rtl8101_eri_write(baseAddr, 0x1B0, 2, 0xED03, ERIAR_ExGMAC);
            mdio_write(tp, 0x1F, 0x0004);
            
            if (ReadReg8(0xEF) & 0x02) {
                mdio_write(tp, 0x10, 0x731F);
                mdio_write(tp, 0x19, 0x7630);
            } else {
                mdio_write(tp, 0x10, 0x711F);
                mdio_write(tp, 0x19, 0x7030);
            }
            mdio_write(tp, 0x1A, 0x1506);
            mdio_write(tp, 0x1B, 0x0551);
            mdio_write(tp, 0x1F, 0x0000);
            mdio_write(tp, 0x0D, 0x0007);
            mdio_write(tp, 0x0E, 0x003C);
            mdio_write(tp, 0x0D, 0x4007);
            mdio_write(tp, 0x0E, 0x0002);
            mdio_write(tp, 0x0D, 0x0000);
            
            mdio_write(tp, 0x1F, 0x0000);
            mdio_write(tp, 0x0D, 0x0003);
            mdio_write(tp, 0x0E, 0x0015);
            mdio_write(tp, 0x0D, 0x4003);
            mdio_write(tp, 0x0E, 0x0002);
            mdio_write(tp, 0x0D, 0x0000);
            break;
            
        case CFG_METHOD_14:
        case CFG_METHOD_15:
        case CFG_METHOD_16:
            rtl8101_eri_write(baseAddr, 0x1B0, 2, 0xED03, ERIAR_ExGMAC);
            mdio_write(tp, 0x1F, 0x0004);
            mdio_write(tp, 0x10, 0x731F);
            mdio_write(tp, 0x19, 0x7630);
            mdio_write(tp, 0x1A, 0x1506);
            mdio_write(tp, 0x1F, 0x0000);
            mdio_write(tp, 0x0D, 0x0007);
            mdio_write(tp, 0x0E, 0x003C);
            mdio_write(tp, 0x0D, 0x4007);
            mdio_write(tp, 0x0E, 0x0002);
            mdio_write(tp, 0x0D, 0x0000);
            break;
            
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            data = rtl8101_eri_read(baseAddr, 0x1B0, 4, ERIAR_ExGMAC);
            data |= BIT_1 | BIT_0;
            rtl8101_eri_write(baseAddr, 0x1B0, 4, data, ERIAR_ExGMAC);
            mdio_write(tp, 0x1F, 0x0A43);
            data = mdio_read(tp, 0x11);
            mdio_write(tp, 0x11, data | BIT_4);
            mdio_write(tp, 0x1F, 0x0A5D);
            mdio_write(tp, 0x10, 0x0006);
            mdio_write(tp, 0x1F, 0x0000);
            break;
    }
    
    switch (tp->mcfg) {
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            mdio_write(tp, 0x1F, 0x0A4A);
            SetEthPhyBit(tp, 0x11, BIT_9);
            mdio_write(tp, 0x1F, 0x0A42);
            SetEthPhyBit(tp, 0x14, BIT_7);
            mdio_write(tp, 0x1F, 0x0000);
            break;
    }
    
    /*Advanced EEE*/
    switch (tp->mcfg) {
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            mdio_write(tp,0x1F, 0x0B82);
            SetEthPhyBit(tp, 0x10, BIT_4);
            mdio_write(tp, 0x1F, 0x0000);
            
            mdio_write(tp,0x1F, 0x0B80);
            WaitCnt = 0;
            do {
                PhyRegValue = mdio_read(tp, 0x10);
                PhyRegValue &= 0x0040;
                udelay(100);
                WaitCnt++;
            } while(PhyRegValue != 0x0040 && WaitCnt <1000);
            
            mdio_write(tp, 0x1F, 0x0000);
            break;
    }
    
    switch (tp->mcfg) {
        case CFG_METHOD_17:
            rtl8101_eri_write(baseAddr, 0x1EA, 1, 0xFA, ERIAR_ExGMAC);
            
            mdio_write(tp, 0x1F, 0x0A43);
            data = mdio_read(tp, 0x10);
            if (data & BIT_10) {
                mdio_write(tp, 0x1F, 0x0A42);
                data = mdio_read(tp, 0x16);
                data &= ~(BIT_1);
                mdio_write(tp, 0x16, data);
            } else {
                mdio_write(tp, 0x1F, 0x0A42);
                data = mdio_read(tp, 0x16);
                data |= BIT_1;
                mdio_write(tp, 0x16, data);
            }
            mdio_write(tp, 0x1F, 0x0000);
            break;
            
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            data = mac_ocp_read(tp, 0xE052);
            data |= BIT_0;
            mac_ocp_write(tp, 0xE052, data);
            
            mdio_write(tp, 0x1F, 0x0A43);
            data = mdio_read(tp, 0x10) | BIT_15;
            mdio_write(tp, 0x10, data);
            
            mdio_write(tp, 0x1F, 0x0A44);
            data = mdio_read(tp, 0x11) | BIT_13 | BIT_14;
            data &= ~(BIT_12);
            mdio_write(tp, 0x11, data);
            break;
    }
    
    switch (tp->mcfg) {
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            mdio_write(tp, 0x1F, 0x0B82);
            ClearEthPhyBit(tp, 0x10, BIT_4);
            mdio_write(tp, 0x1F, 0x0000);
            break;
    }
}

/*
 * Slightly modified version of the Linux driver's rtl8101_disable_EEE()
 * routine.
 */
void RTL8100::disableEEESupport()
{
    struct rtl8101_private *tp = &linuxData;
    UInt16 data;
    UInt16 PhyRegValue;
    UInt32 WaitCnt;
    
    switch (tp->mcfg) {
        case CFG_METHOD_10:
            mdio_write(tp, 0x1F, 0x0007);
            mdio_write(tp, 0x1E, 0x0020);
            data = mdio_read(tp, 0x15) & ~0x0100;
            mdio_write(tp, 0x15, data);
            mdio_write(tp, 0x1F, 0x0006);
            mdio_write(tp, 0x00, 0x5A00);
            mdio_write(tp, 0x1F, 0x0000);
            mdio_write(tp, 0x0D, 0x0007);
            mdio_write(tp, 0x0E, 0x003C);
            mdio_write(tp, 0x0D, 0x4007);
            mdio_write(tp, 0x0E, 0x0000);
            mdio_write(tp, 0x0D, 0x0000);
            mdio_write(tp, 0x1F, 0x0000);

            if (ReadReg8(Config4) & 0x40) {
                mdio_write(tp, 0x1F, 0x0005);
                mdio_write(tp, 0x05, 0x8B82);
                data = mdio_read(tp, 0x06) & ~0x0010;
                mdio_write(tp, 0x05, 0x8B82);
                mdio_write(tp, 0x06, data);
                mdio_write(tp, 0x1F, 0x0000);
            }
            break;
            
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
            rtl8101_eri_write(baseAddr, 0x1B0, 2, 0, ERIAR_ExGMAC);
            mdio_write(tp, 0x1F, 0x0004);
            mdio_write(tp, 0x10, 0x401F);
            mdio_write(tp, 0x19, 0x7030);
            
            mdio_write(tp, 0x1F, 0x0000);
            mdio_write(tp, 0x0D, 0x0007);
            mdio_write(tp, 0x0E, 0x003C);
            mdio_write(tp, 0x0D, 0x4007);
            mdio_write(tp, 0x0E, 0x0000);
            mdio_write(tp, 0x0D, 0x0000);
            
            mdio_write(tp, 0x1F, 0x0000);
            mdio_write(tp, 0x0D, 0x0003);
            mdio_write(tp, 0x0E, 0x0015);
            mdio_write(tp, 0x0D, 0x4003);
            mdio_write(tp, 0x0E, 0x0000);
            mdio_write(tp, 0x0D, 0x0000);
            break;
            
        case CFG_METHOD_14:
            rtl8101_eri_write(baseAddr, 0x1B0, 2, 0, ERIAR_ExGMAC);
            mdio_write(tp, 0x1F, 0x0004);
            mdio_write(tp, 0x10, 0x401F);
            mdio_write(tp, 0x19, 0x7030);
            
            mdio_write(tp, 0x1F, 0x0000);
            mdio_write(tp, 0x0D, 0x0007);
            mdio_write(tp, 0x0E, 0x003C);
            mdio_write(tp, 0x0D, 0x4007);
            mdio_write(tp, 0x0E, 0x0000);
            mdio_write(tp, 0x0D, 0x0000);
            break;
            
        case CFG_METHOD_15:
        case CFG_METHOD_16:
            rtl8101_eri_write(baseAddr, 0x1B0, 2, 0, ERIAR_ExGMAC);
            mdio_write(tp, 0x1F, 0x0004);
            mdio_write(tp, 0x10, 0xC07F);
            mdio_write(tp, 0x19, 0x7030);
            mdio_write(tp, 0x1F, 0x0000);
            
            mdio_write(tp, 0x1F, 0x0000);
            mdio_write(tp, 0x0D, 0x0007);
            mdio_write(tp, 0x0E, 0x003C);
            mdio_write(tp, 0x0D, 0x4007);
            mdio_write(tp, 0x0E, 0x0000);
            mdio_write(tp, 0x0D, 0x0000);
            break;
            
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            data = rtl8101_eri_read(baseAddr, 0x1B0, 4, ERIAR_ExGMAC);
            data &= ~(BIT_1 | BIT_0);
            rtl8101_eri_write(baseAddr, 0x1B0, 4, data, ERIAR_ExGMAC);
            mdio_write(tp, 0x1F, 0x0A43);
            data = mdio_read(tp, 0x11);
            mdio_write(tp, 0x11, data & ~BIT_4);
            mdio_write(tp, 0x1F, 0x0A5D);
            mdio_write(tp, 0x10, 0x0000);
            mdio_write(tp, 0x1F, 0x0000);
            break;
    }
    
    switch (tp->mcfg) {
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            mdio_write(tp, 0x1F, 0x0A42);
            ClearEthPhyBit(tp, 0x14, BIT_7);
            mdio_write(tp, 0x1F, 0x0A4A);
            ClearEthPhyBit(tp, 0x11, BIT_9);
            mdio_write(tp, 0x1F, 0x0000);
            break;
    }
    
    /*Advanced EEE*/
    switch (tp->mcfg) {
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            mdio_write(tp,0x1F, 0x0B82);
            SetEthPhyBit(tp, 0x10, BIT_4);
            mdio_write(tp, 0x1F, 0x0000);
            
            mdio_write(tp,0x1F, 0x0B80);
            WaitCnt = 0;
            do {
                PhyRegValue = mdio_read(tp, 0x10);
                PhyRegValue &= 0x0040;
                udelay(100);
                WaitCnt++;
            } while(PhyRegValue != 0x0040 && WaitCnt <1000);
            
            mdio_write(tp, 0x1F, 0x0000);
            break;
    }
    
    switch (tp->mcfg) {
        case CFG_METHOD_17:
            rtl8101_eri_write(baseAddr, 0x1EA, 1, 0x00, ERIAR_ExGMAC);
            
            mdio_write(tp, 0x1F, 0x0A42);
            data = mdio_read(tp, 0x16);
            data &= ~(BIT_1);
            mdio_write(tp, 0x16, data);
            mdio_write(tp, 0x1F, 0x0000);
            break;
            
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            data = mac_ocp_read(tp, 0xE052);
            data &= ~(BIT_0);
            mac_ocp_write(tp, 0xE052, data);
            
            mdio_write(tp, 0x1F, 0x0A43);
            data = mdio_read(tp, 0x10) & ~(BIT_15);
            mdio_write(tp, 0x10, data);
            
            mdio_write(tp, 0x1F, 0x0A44);
            data = mdio_read(tp, 0x11) & ~(BIT_12 | BIT_13 | BIT_14);
            mdio_write(tp, 0x11, data);
            break;
    }
    
    switch (tp->mcfg) {
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            mdio_write(tp, 0x1F, 0x0B82);
            ClearEthPhyBit(tp, 0x10, BIT_4);
            mdio_write(tp, 0x1F, 0x0000);
            break;
    }
}

/*
 * Called by the watchdog timer task to update interface statistics
 * from the statistics dump done in hardware. Also setup the chip
 * to perform the next statistcis dump.
 */
void RTL8100::updateStatitics()
{
    UInt32 sgColl, mlColl;
    UInt32 cmd;
    
    /* Check if a statistics dump has been completed. */
    if (needsUpdate && !(ReadReg32(CounterAddrLow) & CounterDump)) {
        needsUpdate = false;
        netStats->inputPackets = OSSwapLittleToHostInt64(statData->rxPackets) & 0x00000000ffffffff;
        netStats->inputErrors = OSSwapLittleToHostInt32(statData->rxErrors);
        netStats->outputPackets = OSSwapLittleToHostInt64(statData->txPackets) & 0x00000000ffffffff;
        netStats->outputErrors = OSSwapLittleToHostInt32(statData->txErrors);
        
        sgColl = OSSwapLittleToHostInt32(statData->txOneCollision);
        mlColl = OSSwapLittleToHostInt32(statData->txMultiCollision);
        netStats->collisions = sgColl + mlColl;
        
        etherStats->dot3StatsEntry.singleCollisionFrames = sgColl;
        etherStats->dot3StatsEntry.multipleCollisionFrames = mlColl;
        etherStats->dot3StatsEntry.alignmentErrors = OSSwapLittleToHostInt16(statData->alignErrors);
        etherStats->dot3StatsEntry.missedFrames = OSSwapLittleToHostInt16(statData->rxMissed);
        etherStats->dot3TxExtraEntry.underruns = OSSwapLittleToHostInt16(statData->txUnderun);
    }
    /* Some chips are unable to dump the tally counter while the receiver is disabled. */
    if (ReadReg8(ChipCmd) & CmdRxEnb) {
        WriteReg32(CounterAddrHigh, (statPhyAddr >> 32));
        cmd = (statPhyAddr & 0x00000000ffffffff);
        WriteReg32(CounterAddrLow, cmd);
        WriteReg32(CounterAddrLow, cmd | CounterDump);
        needsUpdate = true;
    }
}

#pragma mark --- hardware initialization methods ---

bool RTL8100::initPCIConfigSpace(IOPCIDevice *provider)
{
    UInt32 pcieLinkCap;
    UInt16 pcieLinkCtl;
    UInt16 cmdReg;
    UInt16 pmCap;
    UInt8 pmCapOffset;
    UInt8 pcieCapOffset;
    bool result = false;
    
    /* Get vendor and device info. */
    pciDeviceData.vendor = provider->configRead16(kIOPCIConfigVendorID);
    pciDeviceData.device = provider->configRead16(kIOPCIConfigDeviceID);
    pciDeviceData.subsystem_vendor = provider->configRead16(kIOPCIConfigSubSystemVendorID);
    pciDeviceData.subsystem_device = provider->configRead16(kIOPCIConfigSubSystemID);
    
    /* Setup power management. */
    if (provider->findPCICapability(kIOPCIPowerManagementCapability, &pmCapOffset)) {
        pmCap = provider->configRead16(pmCapOffset + kIOPCIPMCapability);
        DebugLog("Ethernet [RealtekRTL8100]: PCI power management capabilities: 0x%x.\n", pmCap);
        
        if (pmCap & kPCIPMCPMESupportFromD3Cold) {
            wolCapable = true;
            DebugLog("Ethernet [RealtekRTL8100]: PME# from D3 (cold) supported.\n");
        }
    } else {
        IOLog("Ethernet [RealtekRTL8100]: PCI power management unsupported.\n");
    }
    provider->enablePCIPowerManagement(kPCIPMCSPowerStateD0);
    
    /* Get PCIe link information. */
    if (provider->findPCICapability(kIOPCIPCIExpressCapability, &pcieCapOffset)) {
        pcieLinkCap = provider->configRead32(pcieCapOffset + kIOPCIELinkCapability);
        pcieLinkCtl = provider->configRead16(pcieCapOffset + kIOPCIELinkControl);
        DebugLog("Ethernet [RealtekRTL8100]: PCIe link capabilities: 0x%08x, link control: 0x%04x.\n", pcieLinkCap, pcieLinkCtl);
        
        if (pcieLinkCtl & kIOPCIELinkCtlASPM) {
            if (disableASPM) {
                IOLog("Ethernet [RealtekRTL8111]: Disable PCIe ASPM.\n");
                provider-> setASPMState(this, 0);
            } else {
                IOLog("Ethernet [RealtekRTL8111]: Warning: PCIe ASPM enabled.\n");
                linuxData.aspm = 1;
            }
        }
    }
    /* Enable the device. */
    cmdReg	= provider->configRead16(kIOPCIConfigCommand);
    cmdReg  &= ~kIOPCICommandIOSpace;
    cmdReg	|= (kIOPCICommandBusMaster | kIOPCICommandMemorySpace | kIOPCICommandMemWrInvalidate);
	provider->extendedConfigWrite16(kIOPCIConfigCommand, cmdReg);
    provider->extendedConfigWrite8(kIOPCIConfigLatencyTimer, 0x40);
    provider->extendedConfigWrite32(0x30, 0);

    baseMap = provider->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress2, kIOMapInhibitCache);
    
    if (!baseMap) {
        IOLog("Ethernet [RealtekRTL8100]: region #2 not an MMIO resource, aborting.\n");
        goto done;
    }
    baseAddr = reinterpret_cast<volatile void *>(baseMap->getVirtualAddress());
    linuxData.mmio_addr = baseAddr;
    result = true;
    
done:
    return result;
}

/*
 * Called on wakeup in order to put the chip into power state D0.
 */
IOReturn RTL8100::setPowerStateWakeAction(OSObject *owner, void *arg1, void *arg2, void *arg3, void *arg4)
{
    RTL8100 *ethCtlr = OSDynamicCast(RTL8100, owner);
    
    if (ethCtlr)
        ethCtlr->pciDevice->enablePCIPowerManagement(kPCIPMCSPowerStateD0);
    
    return kIOReturnSuccess;
}

/*
 * Called on sleep in order to put the chip into power state D3 and enable Power Management
 * Events if WoL has been enabled.
 */
IOReturn RTL8100::setPowerStateSleepAction(OSObject *owner, void *arg1, void *arg2, void *arg3, void *arg4)
{
    RTL8100 *ethCtlr = OSDynamicCast(RTL8100, owner);
    IOPCIDevice *dev;
    
    if (ethCtlr) {
        dev = ethCtlr->pciDevice;
        
        if (ethCtlr->wolActive)
            dev->enablePCIPowerManagement(kPCIPMCSPMEStatus | kPCIPMCSPMEEnable | kPCIPMCSPowerStateD3);
        else
            dev->enablePCIPowerManagement(kPCIPMCSPowerStateD3);
    }
    return kIOReturnSuccess;
}

/*
 * Identifies the chip version and performs basic intitialization.
 * Most of the code is based on the Linux driver's rtl8101_init_one()
 * routine.
 * Updating the Linux code will most likely require this method
 * to be rewritten too.
 */
bool RTL8100::initRTL8100()
{
    struct rtl8101_private *tp = &linuxData;
    UInt32 i;
    UInt32 csiTmp;
    UInt16 macAddr[4];
    UInt8 options1, options2;
    bool result = false;
    bool wol;

    /* Soft reset the chip. */
    WriteReg8(ChipCmd, CmdReset);
    
    /* Check that the chip has finished the reset. */
    for (i = 1000; i > 0; i--) {
        if ((ReadReg8(ChipCmd) & CmdReset) == 0)
            break;
        
        IODelay(10);
    }
    /* Identify chip attached to board */
	rtl8101_get_mac_version(tp, baseAddr);
    
    if (tp->mcfg >= CFG_METHOD_MAX) {
        DebugLog("Ethernet [RealtekRTL8100]: Unsupported chip found. Aborting...\n");
        goto done;
    }
    tp->chipset = tp->mcfg;
    
    /* Setup EEE support. */
    if ((tp->mcfg >= CFG_METHOD_10) && enableEEE) {
        eeeAdv = eeeCap = kEEEMode100;
    }
    
    /* Select the chip revision. */
    revision2 = ((tp->chipset == CFG_METHOD_1) || (tp->chipset == CFG_METHOD_2) || (tp->chipset == CFG_METHOD_3)) ? false : true;
    
    tp->set_speed = rtl8101_set_speed_xmii;
    tp->get_settings = rtl8101_gset_xmii;
    tp->phy_reset_enable = rtl8101_xmii_reset_enable;
    tp->phy_reset_pending = rtl8101_xmii_reset_pending;
    tp->link_ok = rtl8101_xmii_link_ok;
    
    /* Setup the interrupt masks for interrupt driven and polled mode. */
    intrMaskRxTx = (SYSErr | RxDescUnavail | TxErr | TxOK | RxErr | RxOK);
    intrMaskPoll = SYSErr;
    intrMask = intrMaskRxTx;

    /* Get the RxConfig parameters. */
    rxConfigMask = rtl_chip_info[tp->chipset].RxConfigMask;
    tp->cp_cmd = ReadReg16(CPlusCmd);
    
    rtl8101_get_bios_setting(tp);
    
    switch (tp->mcfg) {
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            tp->HwSuppNowIsOobVer = 1;
            break;
    }

    if (tp->aspm) {
        switch (tp->mcfg) {
            case CFG_METHOD_15:
            case CFG_METHOD_16:
            case CFG_METHOD_17:
            case CFG_METHOD_18:
            case CFG_METHOD_19:
                tp->org_pci_offset_99 = csiFun0ReadByte(0x99);
                tp->org_pci_offset_99 &= ~(BIT_5|BIT_6);
                break;
        }
        switch (tp->mcfg) {
            case CFG_METHOD_17:
            case CFG_METHOD_18:
            case CFG_METHOD_19:
                tp->org_pci_offset_180 = csiFun0ReadByte(0x180);
                break;
        }
    }
    tp->org_pci_offset_80 = pciDevice->extendedConfigRead8(0x80);
    tp->org_pci_offset_81 = pciDevice->extendedConfigRead8(0x81);
    
    switch (tp->mcfg) {
        case CFG_METHOD_19: {
            u16 ioffset_p3, ioffset_p2, ioffset_p1, ioffset_p0;
            u16 TmpUshort;
            
            mac_ocp_write( tp, 0xDD02, 0x807D);
            TmpUshort = mac_ocp_read( tp, 0xDD02 );
            ioffset_p3 = ( (TmpUshort & BIT_7) >>7 );
            ioffset_p3 <<= 3;
            TmpUshort = mac_ocp_read( tp, 0xDD00 );
            
            ioffset_p3 |= ((TmpUshort & (BIT_15 | BIT_14 | BIT_13))>>13);
            
            ioffset_p2 = ((TmpUshort & (BIT_12|BIT_11|BIT_10|BIT_9))>>9);
            ioffset_p1 = ((TmpUshort & (BIT_8|BIT_7|BIT_6|BIT_5))>>5);
            
            ioffset_p0 = ( (TmpUshort & BIT_4) >>4 );
            ioffset_p0 <<= 3;
            ioffset_p0 |= (TmpUshort & (BIT_2| BIT_1 | BIT_0));
            
            if((ioffset_p3 == 0x0F) && (ioffset_p2 == 0x0F) && (ioffset_p1 == 0x0F) && (ioffset_p0 == 0x0F)) {
                tp->RequireAdcBiasPatch = FALSE;
            } else {
                tp->RequireAdcBiasPatch = TRUE;
                tp->AdcBiasPatchIoffset = (ioffset_p3<<12)|(ioffset_p2<<8)|(ioffset_p1<<4)|(ioffset_p0);
            }
        }
            break;
    }
    
    switch (tp->mcfg) {
        case CFG_METHOD_18:
        case CFG_METHOD_19: {
            u16 rg_saw_cnt;
            
            mdio_write(tp, 0x1F, 0x0C42);
            rg_saw_cnt = mdio_read(tp, 0x13);
            rg_saw_cnt &= ~(BIT_15|BIT_14);
            mdio_write(tp, 0x1F, 0x0000);
            
            if ( rg_saw_cnt > 0) {
                tp->SwrCnt1msIni = 16000000/rg_saw_cnt;
                tp->SwrCnt1msIni &= 0x0FFF;
                
                tp->RequireAdjustUpsTxLinkPulseTiming = TRUE;
            }
        }
            break;
    }
    switch (tp->mcfg) {
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
        case CFG_METHOD_14:
            tp->RequireResetNctlBfrPhyResetOrNway = TRUE;
            break;
    }
    
    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
        case CFG_METHOD_4:
        case CFG_METHOD_5:
        case CFG_METHOD_6:
        case CFG_METHOD_7:
        case CFG_METHOD_8:
        case CFG_METHOD_9:
            tp->RequireResetPhyToChgSpd = TRUE;
            break;
    }
    
    switch (tp->mcfg) {
        case CFG_METHOD_14:
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            tp->HwSuppMagicPktVer = WAKEUP_MAGIC_PACKET_V2;
            break;
        case CFG_METHOD_DEFAULT:
            tp->HwSuppMagicPktVer = WAKEUP_MAGIC_PACKET_NOT_SUPPORT;
            break;
        default:
            tp->HwSuppMagicPktVer = WAKEUP_MAGIC_PACKET_V1;
            break;
    }
    
    switch (tp->mcfg) {
        case CFG_METHOD_17:
            tp->sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_17;
            break;
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            tp->sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_18;
            break;
    }
    
    if (tp->HwIcVerUnknown) {
        tp->NotWrRamCodeToMicroP = TRUE;
        tp->NotWrMcuPatchCode = TRUE;
    }

    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
        case CFG_METHOD_4:
        case CFG_METHOD_5:
        case CFG_METHOD_6:
        case CFG_METHOD_7:
        case CFG_METHOD_8:
        case CFG_METHOD_9:
            tp->RequireResetPhyToChgSpd = TRUE;
            break;
    }
    
    switch (tp->mcfg) {
        case CFG_METHOD_14:
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            tp->HwSuppMagicPktVer = WAKEUP_MAGIC_PACKET_V2;
            break;
        case CFG_METHOD_DEFAULT:
            tp->HwSuppMagicPktVer = WAKEUP_MAGIC_PACKET_NOT_SUPPORT;
            break;
        default:
            tp->HwSuppMagicPktVer = WAKEUP_MAGIC_PACKET_V1;
            break;
    }
    
    switch (tp->mcfg) {
        case CFG_METHOD_17:
            tp->sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_17;
            break;
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            tp->sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_18;
            break;
    }
    
    if (tp->HwIcVerUnknown) {
        tp->NotWrRamCodeToMicroP = TRUE;
        tp->NotWrMcuPatchCode = TRUE;
    }

    rtl8101_exit_oob(tp);
    rtl8101_hw_init(tp);
    rtl8101_nic_reset(tp);
    
    /* Get production from EEPROM */
    if ((tp->mcfg == CFG_METHOD_17 || tp->mcfg == CFG_METHOD_18 ||
         tp->mcfg == CFG_METHOD_19) && (mac_ocp_read(tp, 0xDC00) & BIT_3))
        tp->eeprom_type = EEPROM_TYPE_NONE;
    else
        rtl_eeprom_type(tp);
    
    if (tp->eeprom_type == EEPROM_TYPE_93C46 || tp->eeprom_type == EEPROM_TYPE_93C56)
        rtl_set_eeprom_sel_low(baseAddr);
    
    if (tp->mcfg == CFG_METHOD_14 || tp->mcfg == CFG_METHOD_17 ||
        tp->mcfg == CFG_METHOD_18 || tp->mcfg == CFG_METHOD_19) {
        *(u32*)&macAddr[0] = rtl8101_eri_read(baseAddr, 0xE0, 4, ERIAR_ExGMAC);
        *(u16*)&macAddr[2] = rtl8101_eri_read(baseAddr, 0xE4, 2, ERIAR_ExGMAC);
    } else {
        if (tp->eeprom_type != EEPROM_TYPE_NONE) {
            /* Get MAC address from EEPROM */
            macAddr[0] = rtl_eeprom_read_sc(tp, 7);
            macAddr[1] = rtl_eeprom_read_sc(tp, 8);
            macAddr[2] = rtl_eeprom_read_sc(tp, 9);
            WriteReg8(Cfg9346, Cfg9346_Unlock);
            WriteReg32(MAC0, (macAddr[1] << 16) | macAddr[0]);
            WriteReg16(MAC4, macAddr[2]);
            WriteReg8(Cfg9346, Cfg9346_Lock);
        }
    }
    rtl8101_rar_set(tp, (uint8_t*)macAddr);

	for (i = 0; i < MAC_ADDR_LEN; i++) {
		currMacAddr.bytes[i] = ReadReg8(MAC0 + i);
		origMacAddr.bytes[i] = currMacAddr.bytes[i]; /* keep the original MAC address */
	}
    IOLog("Ethernet [RealtekRTL8100]: %s: (Chipset %d) at 0x%p, %2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x\n",
          rtl_chip_info[tp->chipset].name, tp->chipset, baseAddr,
          origMacAddr.bytes[0], origMacAddr.bytes[1],
          origMacAddr.bytes[2], origMacAddr.bytes[3],
          origMacAddr.bytes[4], origMacAddr.bytes[5]);
    
    /*
     * Determine the chip's WoL capabilities. Most of the code is
     * taken from the linux driver's rtl8101_get_hw_wol() routine.
     */
    options1 = ReadReg8(Config3);
    options2 = ReadReg8(Config5);
    
    if (options1 & LinkUp)
        tp->wol_opts |= WAKE_PHY;
    
    switch (tp->HwSuppMagicPktVer) {
        case WAKEUP_MAGIC_PACKET_V2:
            csiTmp = rtl8101_eri_read(baseAddr, 0xDE, 1, ERIAR_ExGMAC);
            
            if (csiTmp & BIT_0)
                tp->wol_opts |= WAKE_MAGIC;
            break;
            
        default:
            if (options1 & MagicPacket)
                tp->wol_opts |= WAKE_MAGIC;
            break;
    }
    if (options2 & UWF)
        tp->wol_opts |= WAKE_UCAST;
    
    if (options2 & BWF)
        tp->wol_opts |= WAKE_BCAST;
    
    if (options2 & MWF)
        tp->wol_opts |= WAKE_MCAST;
    
    wol = ((options1 & (LinkUp | MagicPacket)) || (options2 & (UWF | BWF | MWF))) ? true : false;
    
    /* Set wake on LAN support. */
    wolCapable = wolCapable && wol;

    result = true;
    
done:
    return result;
}

void RTL8100::enableRTL8100()
{
    struct rtl8101_private *tp = &linuxData;
    
    setLinkStatus(kIONetworkLinkValid);

    intrMask = intrMaskRxTx;
    polling = false;

    rtl8101_exit_oob(tp);
    rtl8101_hw_init(tp);
    rtl8101_nic_reset(tp);
    rtl8101_powerup_pll(tp);
    rtl8101_hw_ephy_config(tp);
    rtl8101_hw_phy_config(tp);
	startRTL8100(intrMitigateValue, true);
	rtl8101_dsm(tp, DSM_IF_UP);

    setPhyMedium();
}

void RTL8100::disableRTL8100()
{
    struct rtl8101_private *tp = &linuxData;
    
	rtl8101_dsm(tp, DSM_IF_DOWN);
    
    /* Disable all interrupts by clearing the interrupt mask. */
    WriteReg16(IntrMask, 0);
    WriteReg16(IntrStatus, ReadReg16(IntrStatus));

    rtl8101_nic_reset(tp);
    
    hardwareD3Para();
	powerdownPLL();
    rtl8101_set_bios_setting(tp);
    
    if (linkUp) {
        linkUp = false;
        setLinkStatus(kIONetworkLinkValid);
        IOLog("Ethernet [RealtekRTL8100]: Link down on en%u\n", netif->getUnitNumber());
    }

}

/* Resets the NIC in case a tx deadlock or a pci error occurred. timerSource and txQueue
 * are stopped immediately but will be restarted by the timer task when the link has
 * been reestablished.
 */

void RTL8100::restartRTL8100()
{
    /* Stop output thread and flush txQueue */
    netif->stopOutputThread();
    netif->flushOutputQueue();
    linkUp = false;
    setLinkStatus(kIONetworkLinkValid);
    
    /* Reset NIC and cleanup both descriptor rings. */
    rtl8101_nic_reset(&linuxData);
    txClearDescriptors();
    
    if (rxInterrupt(netif, kNumRxDesc, NULL, NULL))
        netif->flushInputQueue();
    
    rxNextDescIndex = 0;
    deadlockWarn = 0;
    
    /* Reinitialize NIC. */
    enableRTL8100();
}

/*
 * This is a rewrite of the linux driver's rtl8101_hw_start() routine.
 *
 * Updating the Linux code will most likely require this method
 * to be rewritten too.
 */
void RTL8100::startRTL8100(UInt16 newIntrMitigate, bool enableInterrupts)
{
    struct rtl8101_private *tp = &linuxData;
    UInt32 csiTmp;
    UInt16 mac_ocp_data;
    UInt8 link_control;
    
    WriteReg32(RxConfig, (RX_DMA_BURST << RxCfgDMAShift));
    
    rtl8101_nic_reset(tp);
    
    WriteReg8(Cfg9346, Cfg9346_Unlock);
    
    switch (tp->mcfg) {
        case CFG_METHOD_10:
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
        case CFG_METHOD_14:
        case CFG_METHOD_15:
        case CFG_METHOD_16:
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            WriteReg8(Config5, ReadReg8(Config5) & ~BIT_0);
            WriteReg8(Config2, ReadReg8(Config2) & ~BIT_7);
            WriteReg8(0xF1, ReadReg8(0xF1) & ~BIT_7);
            break;
    }
    //clear io_rdy_l23
    switch (tp->mcfg) {
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
        case CFG_METHOD_14:
        case CFG_METHOD_15:
        case CFG_METHOD_16:
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            WriteReg8(Config3, ReadReg8(Config3) & ~BIT_1);
            break;
    }
    WriteReg8(MTPS, Reserved1_data);
    
    /* Set DMA burst size and Interframe Gap Time */
    WriteReg32(TxConfig, (TX_DMA_BURST << TxDMAShift) | (InterFrameGap << TxInterFrameGapShift));
    
    tp->cp_cmd &= 0x2063;
    
    WriteReg16(IntrMitigate, intrMitigateValue);
    
    WriteReg32(TxDescStartAddrLow, (UInt32)(txPhyAddr & 0x00000000ffffffff));
    WriteReg32(TxDescStartAddrHigh, (UInt32)(txPhyAddr >> 32));
    WriteReg32(RxDescAddrLow, (UInt32)(rxPhyAddr & 0x00000000ffffffff));
    WriteReg32(RxDescAddrHigh, (UInt32)(rxPhyAddr >> 32));
    
    if (tp->mcfg == CFG_METHOD_4) {
        set_offset70F(tp, 0x17);
        setOffset79(0x50);
        
        link_control = pciDevice->extendedConfigRead8(0x81);
        
        if (link_control == 1) {
            pciDevice->extendedConfigWrite8(0x81, 0);
            
            WriteReg8(DBG_reg, 0x98);
            WriteReg8(Config2, ReadReg8(Config2) | BIT_7);
            WriteReg8(Config4, ReadReg8(Config4) | BIT_2);
            
            pciDevice->extendedConfigWrite8(0x81, 1);
        }
        
        WriteReg8(Config1, 0x0f);
        
        WriteReg8(Config3, ReadReg8(Config3) & ~Beacon_en);
    } else if (tp->mcfg == CFG_METHOD_5) {
        link_control = pciDevice->extendedConfigRead8(0x81);
        
        if (link_control == 1) {
            pciDevice->extendedConfigWrite8(0x81, 0);
            
            WriteReg8(DBG_reg, 0x98);
            WriteReg8(Config2, ReadReg8(Config2) | BIT_7);
            WriteReg8(Config4, ReadReg8(Config4) | BIT_2);
            
            pciDevice->extendedConfigWrite8(0x81, 1);
        }
        
        setOffset79(0x50);
        
        WriteReg8(Config1, 0x0f);
        
        WriteReg8(Config3, ReadReg8(Config3) & ~Beacon_en);
    } else if (tp->mcfg == CFG_METHOD_6) {
        link_control = pciDevice->extendedConfigRead8(0x81);
        
        if (link_control == 1) {
            pciDevice->extendedConfigWrite8(0x81, 0);
            
            WriteReg8(DBG_reg, 0x98);
            WriteReg8(Config2, ReadReg8(Config2) | BIT_7);
            WriteReg8(Config4, ReadReg8(Config4) | BIT_2);
            
            pciDevice->extendedConfigWrite8(0x81, 1);
        }
        
        setOffset79(0x50);
        
        //		WriteReg8(Config1, 0xDF);
        
        WriteReg8(0xF4, 0x01);
        
        WriteReg8(Config3, ReadReg8(Config3) & ~Beacon_en);
    } else if (tp->mcfg == CFG_METHOD_7) {
        link_control = pciDevice->extendedConfigRead8(0x81);
        
        if (link_control == 1) {
            pciDevice->extendedConfigWrite8(0x81, 0);
            
            WriteReg8(DBG_reg, 0x98);
            WriteReg8(Config2, ReadReg8(Config2) | BIT_7);
            WriteReg8(Config4, ReadReg8(Config4) | BIT_2);
            
            pciDevice->extendedConfigWrite8(0x81, 1);
        }
        
        setOffset79(0x50);
        
        //		WriteReg8(Config1, (ReadReg8(Config1)&0xC0)|0x1F);
        
        WriteReg8(0xF4, 0x01);
        
        WriteReg8(Config3, ReadReg8(Config3) & ~Beacon_en);
        
        WriteReg8(0xF5, ReadReg8(0xF5) | BIT_2);
    } else if (tp->mcfg == CFG_METHOD_8) {
        link_control = pciDevice->extendedConfigRead8(0x81);
        
        if (link_control == 1) {
            pciDevice->extendedConfigWrite8(0x81, 0);
            
            WriteReg8(DBG_reg, 0x98);
            WriteReg8(Config2, ReadReg8(Config2) | BIT_7);
            WriteReg8(Config4, ReadReg8(Config4) | BIT_2);
            WriteReg8(0xF4, ReadReg8(0xF4) | BIT_3);
            WriteReg8(0xF5, ReadReg8(0xF5) | BIT_2);
            
            pciDevice->extendedConfigWrite8(0x81, 1);
            
            if (rtl8101_ephy_read(baseAddr, 0x10)==0x0008) {
                rtl8101_ephy_write(baseAddr, 0x10, 0x000C);
            }
        }
        
        link_control = pciDevice->extendedConfigRead8(0x80);
        
        if (link_control & 3)
            rtl8101_ephy_write(baseAddr, 0x02, 0x011F);
        
        setOffset79(0x50);
        
        //		WriteReg8(Config1, (ReadReg8(Config1)&0xC0)|0x1F);
        
        WriteReg8(0xF4, ReadReg8(0xF4) | BIT_0);
        
        WriteReg8(Config3, ReadReg8(Config3) & ~Beacon_en);
    } else if (tp->mcfg == CFG_METHOD_9) {
        link_control = pciDevice->extendedConfigRead8(0x81);
        
        if (link_control == 1) {
            pciDevice->extendedConfigWrite8(0x81, 0);
            
            WriteReg8(DBG_reg, 0x98);
            WriteReg8(Config2, ReadReg8(Config2) | BIT_7);
            WriteReg8(Config4, ReadReg8(Config4) | BIT_2);
            
            pciDevice->extendedConfigWrite8(0x81, 1);
        }
        
        setOffset79(0x50);
        
        //		WriteReg8(Config1, 0xDF);
        
        WriteReg8(0xF4, 0x01);
        
        WriteReg8(Config3, ReadReg8(Config3) & ~Beacon_en);
    } else if (tp->mcfg == CFG_METHOD_10) {
        set_offset70F(tp, 0x27);
        setOffset79(0x50);
        
        WriteReg8(0xF3, ReadReg8(0xF3) | BIT_5);
        WriteReg8(0xF3, ReadReg8(0xF3) & ~BIT_5);
        
        WriteReg8(0xD0, ReadReg8(0xD0) | BIT_7 | BIT_6);
        
        WriteReg8(0xF1, ReadReg8(0xF1) | BIT_6 | BIT_5 | BIT_4 | BIT_2 | BIT_1);
        
        if (tp->aspm)
            WriteReg8(0xF1, ReadReg8(0xF1) | BIT_7);
        
        WriteReg8(Config5, (ReadReg8(Config5)&~0x08) | BIT_0);
        WriteReg8(Config2, ReadReg8(Config2) | BIT_7);
        
        WriteReg8(Config3, ReadReg8(Config3) & ~Beacon_en);
    } else if (tp->mcfg == CFG_METHOD_11 || tp->mcfg == CFG_METHOD_12 ||
               tp->mcfg == CFG_METHOD_13) {
        u8	pci_config;
        
        tp->cp_cmd &= 0x2063;
        
        pci_config = pciDevice->extendedConfigRead8(0x80);
        
        if (pci_config & 0x03) {
            WriteReg8(Config5, ReadReg8(Config5) | BIT_0);
            WriteReg8(0xF2, ReadReg8(0xF2) | BIT_7);
            if (tp->aspm)
                WriteReg8(0xF1, ReadReg8(0xF1) | BIT_7);
            
            WriteReg8(Config2, ReadReg8(Config2) | BIT_7);
        }
        
        WriteReg8(0xF1, ReadReg8(0xF1) | BIT_5 | BIT_3);
        WriteReg8(0xF2, ReadReg8(0xF2) & ~BIT_0);
        WriteReg8(0xD3, ReadReg8(0xD3) | BIT_3 | BIT_2);
        WriteReg8(0xD0, ReadReg8(0xD0) | BIT_6);
        WriteReg16(0xE0, ReadReg16(0xE0) & ~0xDF9C);
        
        if (tp->mcfg == CFG_METHOD_11)
            WriteReg8(Config5, ReadReg8(Config5) & ~BIT_0);
    } else if (tp->mcfg == CFG_METHOD_14) {
        set_offset70F(tp, 0x27);
        setOffset79(0x50);
        
        rtl8101_eri_write(baseAddr, 0xC8, 4, 0x00000002, ERIAR_ExGMAC);
        rtl8101_eri_write(baseAddr, 0xE8, 4, 0x00000006, ERIAR_ExGMAC);
        WriteReg32(TxConfig, ReadReg32(TxConfig) | BIT_7);
        WriteReg8(0xD3, ReadReg8(0xD3) & ~BIT_7);
        csiTmp = rtl8101_eri_read(baseAddr, 0xDC, 1, ERIAR_ExGMAC);
        csiTmp &= ~BIT_0;
        rtl8101_eri_write( baseAddr, 0xDC, 1, csiTmp, ERIAR_ExGMAC);
        csiTmp |= BIT_0;
        rtl8101_eri_write( baseAddr, 0xDC, 1, csiTmp, ERIAR_ExGMAC);
        
        rtl8101_ephy_write(baseAddr, 0x19, 0xff64);
        
        WriteReg8(Config5, ReadReg8(Config5) | BIT_0);
        WriteReg8(Config2, ReadReg8(Config2) | BIT_7);
        
        rtl8101_eri_write(baseAddr, 0xC0, 2, 0x00000000, ERIAR_ExGMAC);
        rtl8101_eri_write(baseAddr, 0xB8, 2, 0x00000000, ERIAR_ExGMAC);
        rtl8101_eri_write(baseAddr, 0xD5, 1, 0x0000000E, ERIAR_ExGMAC);
    } else if (tp->mcfg == CFG_METHOD_15 || tp->mcfg == CFG_METHOD_16) {
        u8	pci_config;
        
        tp->cp_cmd &= 0x2063;
        
        pci_config = pciDevice->extendedConfigRead8(0x80);
        
        if (pci_config & 0x03) {
            WriteReg8(Config5, ReadReg8(Config5) | BIT_0);
            WriteReg8(0xF2, ReadReg8(0xF2) | BIT_7);
            if (tp->aspm)
                WriteReg8(0xF1, ReadReg8(0xF1) | BIT_7);
            WriteReg8(Config2, ReadReg8(Config2) | BIT_7);
        }
        
        WriteReg8(0xF1, ReadReg8(0xF1) | BIT_5 | BIT_3);
        WriteReg8(0xF2, ReadReg8(0xF2) & ~BIT_0);
        WriteReg8(0xD3, ReadReg8(0xD3) | BIT_3 | BIT_2);
        WriteReg8(0xD0, ReadReg8(0xD0) & ~BIT_6);
        WriteReg16(0xE0, ReadReg16(0xE0) & ~0xDF9C);
    } else if (tp->mcfg == CFG_METHOD_17 || tp->mcfg == CFG_METHOD_18 ||
               tp->mcfg == CFG_METHOD_19) {
        set_offset70F(tp, 0x17);
        setOffset79(0x50);
        
        rtl8101_eri_write(baseAddr, 0xC8, 4, 0x00080002, ERIAR_ExGMAC);
        rtl8101_eri_write(baseAddr, 0xCC, 1, 0x38, ERIAR_ExGMAC);
        rtl8101_eri_write(baseAddr, 0xD0, 1, 0x48, ERIAR_ExGMAC);
        rtl8101_eri_write(baseAddr, 0xE8, 4, 0x00100006, ERIAR_ExGMAC);
        
        WriteReg32(TxConfig, ReadReg32(TxConfig) | BIT_7);
        
        csiTmp = rtl8101_eri_read(baseAddr, 0xDC, 1, ERIAR_ExGMAC);
        csiTmp &= ~BIT_0;
        rtl8101_eri_write(baseAddr, 0xDC, 1, csiTmp, ERIAR_ExGMAC);
        csiTmp |= BIT_0;
        rtl8101_eri_write(baseAddr, 0xDC, 1, csiTmp, ERIAR_ExGMAC);
        
        if (tp->mcfg == CFG_METHOD_18 || tp->mcfg == CFG_METHOD_19) {
            if(tp->RequireAdjustUpsTxLinkPulseTiming) {
                mac_ocp_data = mac_ocp_read(tp, 0xD412);
                mac_ocp_data &= ~(0x0FFF);
                mac_ocp_data |= tp->SwrCnt1msIni ;
                mac_ocp_write(tp, 0xD412, mac_ocp_data);
            }
            
            mac_ocp_data = mac_ocp_read(tp, 0xE056);
            mac_ocp_data &= ~(BIT_7 | BIT_6 | BIT_5 | BIT_4);
            mac_ocp_data |= (BIT_6 | BIT_5 | BIT_4);
            mac_ocp_write(tp, 0xE056, mac_ocp_data);
            
            mac_ocp_data = mac_ocp_read(tp, 0xE052);
            mac_ocp_data &= ~( BIT_14 | BIT_13);
            mac_ocp_data |= BIT_15;
            mac_ocp_data |= BIT_3;
            mac_ocp_write(tp, 0xE052, mac_ocp_data);
            
            mac_ocp_data = mac_ocp_read(tp, 0xD420);
            mac_ocp_data &= ~(BIT_11 | BIT_10 | BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4 | BIT_3 | BIT_2 | BIT_1 | BIT_0);
            mac_ocp_data |= 0x47F;
            mac_ocp_write(tp, 0xD420, mac_ocp_data);
            
            mac_ocp_data = mac_ocp_read(tp, 0xE0D6);
            mac_ocp_data &= ~(BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4 | BIT_3 | BIT_2 | BIT_1 | BIT_0);
            mac_ocp_data |= 0x17F;
            mac_ocp_write(tp, 0xE0D6, mac_ocp_data);
        }
        
        WriteReg8(Config3, ReadReg8(Config3) & ~Beacon_en);
        
        tp->cp_cmd = ReadReg16(CPlusCmd) &
        ~(EnableBist | Macdbgo_oe | Force_halfdup |
          Force_rxflow_en | Force_txflow_en |
          Cxpl_dbg_sel | ASF | PktCntrDisable |
          Macdbgo_sel);
        
        WriteReg8(0x1B, ReadReg8(0x1B) & ~0x07);
        
        WriteReg8(TDFNR, 0x4);
        
        if (tp->aspm)
            WriteReg8(0xF1, ReadReg8(0xF1) | BIT_7);
        
        WriteReg8(0xD0, ReadReg8(0xD0) | BIT_6);
        WriteReg8(0xF2, ReadReg8(0xF2) | BIT_6);
        
        WriteReg8(0xD0, ReadReg8(0xD0) | BIT_7);
        
        rtl8101_eri_write(baseAddr, 0xC0, 2, 0x0000, ERIAR_ExGMAC);
        rtl8101_eri_write(baseAddr, 0xB8, 4, 0x00000000, ERIAR_ExGMAC);
        
        rtl8101_eri_write(baseAddr, 0x5F0, 2, 0x4F87, ERIAR_ExGMAC);
        
        if (tp->mcfg == CFG_METHOD_18 || tp->mcfg == CFG_METHOD_19) {
            csiTmp = rtl8101_eri_read(baseAddr, 0xD4, 4, ERIAR_ExGMAC);
            csiTmp |= (BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_12);
            rtl8101_eri_write(baseAddr, 0xD4, 4, csiTmp, ERIAR_ExGMAC);
            
            csiTmp = rtl8101_eri_read(baseAddr, 0xDC, 4, ERIAR_ExGMAC);
            csiTmp |= (BIT_2 | BIT_3 | BIT_4);
            rtl8101_eri_write(baseAddr, 0xDC, 4, csiTmp, ERIAR_ExGMAC);
        } else {
            csiTmp = rtl8101_eri_read(baseAddr, 0xD4, 4, ERIAR_ExGMAC);
            csiTmp |= (BIT_7 | BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_12);
            rtl8101_eri_write(baseAddr, 0xD4, 4, csiTmp, ERIAR_ExGMAC);
        }
        
        if (tp->mcfg == CFG_METHOD_17) {
            mac_ocp_write(tp, 0xC140, 0xFFFF);
        } else if (tp->mcfg == CFG_METHOD_18 || tp->mcfg == CFG_METHOD_19) {
            mac_ocp_write(tp, 0xC140, 0xFFFF);
            mac_ocp_write(tp, 0xC142, 0xFFFF);
        }
        
        csiTmp = rtl8101_eri_read(baseAddr, 0x1B0, 4, ERIAR_ExGMAC);
        csiTmp &= ~BIT_12;
        rtl8101_eri_write(baseAddr, 0x1B0, 4, csiTmp, ERIAR_ExGMAC);
        
        if (tp->mcfg == CFG_METHOD_18 || tp->mcfg == CFG_METHOD_19) {
            csiTmp = rtl8101_eri_read(baseAddr, 0x2FC, 1, ERIAR_ExGMAC);
            csiTmp &= ~(BIT_2);
            rtl8101_eri_write(baseAddr, 0x2FC, 1, csiTmp, ERIAR_ExGMAC);
        } else {
            csiTmp = rtl8101_eri_read(baseAddr, 0x2FC, 1, ERIAR_ExGMAC);
            csiTmp &= ~(BIT_0 | BIT_1 | BIT_2);
            csiTmp |= BIT_0;
            rtl8101_eri_write(baseAddr, 0x2FC, 1, csiTmp, ERIAR_ExGMAC);
        }
        
        csiTmp = rtl8101_eri_read(baseAddr, 0x1D0, 1, ERIAR_ExGMAC);
        csiTmp |= BIT_1;
        rtl8101_eri_write(baseAddr, 0x1D0, 1, csiTmp, ERIAR_ExGMAC);
    }
    
    //other hw parameters
    if (tp->mcfg == CFG_METHOD_17)
        rtl8101_eri_write(baseAddr, 0x2F8, 2, 0x1D8F, ERIAR_ExGMAC);
    
    if (tp->bios_setting & BIT_28) {
        if (tp->mcfg == CFG_METHOD_13) {
            if (ReadReg8(0xEF) & BIT_2) {
                u32 gphy_val;
                
                spin_lock_irqsave(&tp->phy_lock, flags);
                mdio_write(tp, 0x1F, 0x0001);
                gphy_val = mdio_read(tp, 0x1B);
                gphy_val |= BIT_2;
                mdio_write(tp, 0x1B, gphy_val);
                mdio_write(tp, 0x1F, 0x0000);
                spin_unlock_irqrestore(&tp->phy_lock, flags);
            }
        }
        
        if (tp->mcfg == CFG_METHOD_14) {
            u32 gphy_val;
            
            spin_lock_irqsave(&tp->phy_lock, flags);
            mdio_write(tp, 0x1F, 0x0001);
            gphy_val = mdio_read(tp, 0x13);
            gphy_val |= BIT_15;
            mdio_write(tp, 0x13, gphy_val);
            mdio_write(tp, 0x1F, 0x0000);
            spin_unlock_irqrestore(&tp->phy_lock, flags);
        }
    }
    
    switch (tp->mcfg) {
        case CFG_METHOD_17:
            mac_ocp_write(tp, 0xD3C0, 0x03F8);
            mac_ocp_write(tp, 0xD3C2, 0x0000);
            break;
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            mac_ocp_write(tp, 0xE098, 0x0AA2);
            break;
    }
    
    switch (tp->mcfg) {
        case CFG_METHOD_15:
        case CFG_METHOD_16:
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            if (tp->aspm)
                initPCIOffset99(tp);
            break;
    }
    
    switch (tp->mcfg) {
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            if (tp->aspm)
                rtl8101_init_pci_offset_180(tp);
            break;
    }
    tp->cp_cmd |= (RxChkSum | RxVlan);
    WriteReg16(CPlusCmd, tp->cp_cmd);
    ReadReg16(CPlusCmd);

    switch (tp->mcfg) {
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            {
                int timeout;
                for (timeout = 0; timeout < 10; timeout++) {
                    if ((rtl8101_eri_read(baseAddr, 0x1AE, 2, ERIAR_ExGMAC) & BIT_13)==0)
                    break;
                    mdelay(1);
                }
            }
            break;
    }
    switch (tp->mcfg) {
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
        case CFG_METHOD_14:
        case CFG_METHOD_15:
        case CFG_METHOD_16:
        case CFG_METHOD_17:
            WriteReg16(RxMaxSize, 0x05F3);
            break;
            
        default:
            WriteReg16(RxMaxSize, 0x05EF);
            break;
    }
    rtl8101_disable_rxdvgate(tp);
    rtl8101_dsm(tp, DSM_MAC_INIT);
    
    tp->wol_enabled = (wolCapable && wolActive) ? WOL_ENABLED : WOL_DISABLED;

    /* Set receiver mode. */
    setMulticastMode(multicastMode);
    
    switch (tp->mcfg) {
        case CFG_METHOD_10:
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
        case CFG_METHOD_14:
        case CFG_METHOD_15:
        case CFG_METHOD_16:
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            if (tp->aspm) {
                WriteReg8(Config5, ReadReg8(Config5) | BIT_0);
                WriteReg8(Config2, ReadReg8(Config2) | BIT_7);
            } else {
                WriteReg8(Config5, ReadReg8(Config5) & ~BIT_0);
                WriteReg8(Config2, ReadReg8(Config2) & ~BIT_7);
            }
            break;
    }
    WriteReg8(Cfg9346, Cfg9346_Lock);
    WriteReg8(ChipCmd, CmdTxEnb | CmdRxEnb);
    
    /* Enable all known interrupts by setting the interrupt mask. */
    WriteReg16(IntrMask, intrMask);

    IODelay(10);
}

/*
 * Set PCI configuration space offset 0x79 to setting.
 *
 * Corresponds to set_offset79() in the Linux code.
 *
 * Updating the Linux code will most likely require this method
 * to be rewritten too.
 */
void RTL8100::setOffset79(UInt8 setting)
{
    UInt8 deviceControl;
    
    DebugLog("setOffset79() ===>\n");
    
    deviceControl = pciDevice->configRead8(0x79);
    deviceControl &= ~0x70;
    deviceControl |= setting;
    pciDevice->extendedConfigWrite8(0x79, deviceControl);
    
    DebugLog("setOffset79() <===\n");
}

/*
 * Corresponds to rtl8101_csi_fun0_read_byte() in the Linux code.
 *
 * Updating the Linux code will most likely require this method
 * to be rewritten too.
 */
UInt8 RTL8100::csiFun0ReadByte(UInt32 addr)
{
    UInt8 retVal = 0;
    
    if (linuxData.mcfg == CFG_METHOD_14) {
        UInt32 tmpUlong;
        UInt16 regAlignAddr;
        UInt8 shiftByte;
        
        regAlignAddr = addr & ~(0x3);
        shiftByte = addr & (0x3);
        tmpUlong = rtl8101_csi_other_fun_read(&linuxData, 0, addr);
        tmpUlong >>= (8 * shiftByte);
        retVal = (UInt8)tmpUlong;
    } else {
        retVal = pciDevice->extendedConfigRead8(addr);
    }
    udelay(20);
    
    return retVal;
}

/*
 * Corresponds to rtl8101_csi_fun0_write_byte() in the Linux code.
 *
 * Updating the Linux code will most likely require this method
 * to be rewritten too.
 */
void RTL8100::csiFun0WriteByte(UInt32 addr, UInt8 value)
{
    if (linuxData.mcfg == CFG_METHOD_14) {
        UInt32 tmpUlong;
        UInt16 regAlignAddr;
        UInt8 shiftByte;
        
        regAlignAddr = addr & ~(0x3);
        shiftByte = addr & (0x3);
        tmpUlong = rtl8101_csi_other_fun_read(&linuxData, 0, regAlignAddr);
        tmpUlong &= ~(0xFF << (8 * shiftByte));
        tmpUlong |= (value << (8 * shiftByte));
        rtl8101_csi_other_fun_write(&linuxData, 0, regAlignAddr, tmpUlong );
    } else {
        pciDevice->extendedConfigWrite8(addr, value);
    }
    udelay(20);
}

/*
 * Corresponds to rtl8101_disable_pci_offset_99() in the Linux code.
 *
 * Updating the Linux code will most likely require this method
 * to be rewritten too.
 */
void RTL8100::disablePCIOffset99()
{
    u32 csiTmp;
    
    switch (linuxData.mcfg) {
        case CFG_METHOD_15:
        case CFG_METHOD_16:
            csiTmp = rtl8101_eri_read(baseAddr, 0xC0, 2, ERIAR_ExGMAC);
            csiTmp &= ~(BIT_0 | BIT_1);
            rtl8101_eri_write(baseAddr, 0xC0, 2, csiTmp, ERIAR_ExGMAC);
            break;
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            csiTmp = rtl8101_eri_read(baseAddr, 0x3F2, 2, ERIAR_ExGMAC);
            csiTmp &= ~(BIT_0 | BIT_1);
            rtl8101_eri_write(baseAddr, 0x3F2, 2, csiTmp, ERIAR_ExGMAC);
            break;
    }
    
    switch (linuxData.mcfg) {
        case CFG_METHOD_15:
        case CFG_METHOD_16:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            csiFun0WriteByte(0x99, 0x00);
            break;
    }
}

/*
 * Corresponds to rtl8101_enable_pci_offset_99() in the Linux code.
 *
 * Updating the Linux code will most likely require this method
 * to be rewritten too.
 */
void RTL8100::enablePCIOffset99()
{
    u32 csiTmp;
    
    switch (linuxData.mcfg) {
        case CFG_METHOD_15:
        case CFG_METHOD_16:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            csiFun0WriteByte(0x99, linuxData.org_pci_offset_99);
            break;
    }
    
    switch (linuxData.mcfg) {
        case CFG_METHOD_15:
        case CFG_METHOD_16:
            csiTmp = rtl8101_eri_read(baseAddr, 0xC0, 2, ERIAR_ExGMAC);
            csiTmp &= ~(BIT_0 | BIT_1);
            
            if (!(linuxData.org_pci_offset_99 & (BIT_5 | BIT_6)))
                csiTmp |= BIT_1;
                
            if (!(linuxData.org_pci_offset_99 & BIT_2))
                csiTmp |= BIT_0;
            rtl8101_eri_write(baseAddr, 0xC0, 2, csiTmp, ERIAR_ExGMAC);
            break;
            
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            csiTmp = rtl8101_eri_read(baseAddr, 0x3F2, 2, ERIAR_ExGMAC);
            csiTmp &= ~(BIT_0 | BIT_1);
            
            if (!(linuxData.org_pci_offset_99 & (BIT_5 | BIT_6)))
                csiTmp |= BIT_1;
            
            if (!(linuxData.org_pci_offset_99 & BIT_2))
                csiTmp |= BIT_0;
            
            rtl8101_eri_write(baseAddr, 0x3F2, 2, csiTmp, ERIAR_ExGMAC);
            break;
    }
}

/*
 * Corresponds to rtl8101_init_pci_offset_99() in the Linux code.
 *
 * Updating the Linux code will most likely require this method
 * to be rewritten too.
 */
void RTL8100::initPCIOffset99(struct rtl8101_private *tp)
{
    u32 csiTmp;
    
    switch (linuxData.mcfg) {
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            csiTmp = rtl8101_eri_read(baseAddr, 0x3F2, 2, ERIAR_ExGMAC);
            csiTmp &= ~( BIT_8 | BIT_9  | BIT_10 | BIT_11  | BIT_12  | BIT_13  | BIT_14 | BIT_15 );
            csiTmp |= ( BIT_9 | BIT_10 | BIT_13  | BIT_14 | BIT_15 );
            rtl8101_eri_write(baseAddr, 0x3F2, 2, csiTmp, ERIAR_ExGMAC);
            csiTmp = rtl8101_eri_read(baseAddr, 0x3F5, 1, ERIAR_ExGMAC);
            csiTmp |= BIT_6 | BIT_7;
            rtl8101_eri_write(baseAddr, 0x3F5, 1, csiTmp, ERIAR_ExGMAC);
            mac_ocp_write(tp, 0xE02C, 0x1880);
            mac_ocp_write(tp, 0xE02E, 0x4880);
            break;
    }
    
    switch (linuxData.mcfg) {
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            if (tp->org_pci_offset_99 & BIT_2) {
                csiTmp = rtl8101_eri_read(baseAddr, 0x5C8, 1, ERIAR_ExGMAC);
                csiTmp |= BIT_0;
                rtl8101_eri_write(baseAddr, 0x5C8, 1, csiTmp, ERIAR_ExGMAC);
            }
            break;
    }
    
    switch (linuxData.mcfg) {
        case CFG_METHOD_15:
        case CFG_METHOD_16:
            csiTmp = rtl8101_eri_read(baseAddr, 0xC0, 1, ERIAR_ExGMAC);
            csiTmp |= ( BIT_2 | BIT_3 );
            rtl8101_eri_write(baseAddr, 0xC0, 1, csiTmp, ERIAR_ExGMAC);
            
            rtl8101_eri_write(baseAddr, 0xC8, 2, 0x8C12, ERIAR_ExGMAC);
            rtl8101_eri_write(baseAddr, 0xCA, 2, 0x9003, ERIAR_ExGMAC);
            rtl8101_eri_write(baseAddr, 0xCC, 2, 0x9003, ERIAR_ExGMAC);
            rtl8101_eri_write(baseAddr, 0xCE, 2, 0x9003, ERIAR_ExGMAC);
            break;
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            rtl8101_eri_write(baseAddr, 0x2E8, 2, 0x9003, ERIAR_ExGMAC);
            rtl8101_eri_write(baseAddr, 0x2EA, 2, 0x9003, ERIAR_ExGMAC);
            rtl8101_eri_write(baseAddr, 0x2EC, 2, 0x9003, ERIAR_ExGMAC);
            rtl8101_eri_write(baseAddr, 0x2E2, 2, 0x883C, ERIAR_ExGMAC);
            rtl8101_eri_write(baseAddr, 0x2E4, 2, 0x8C12, ERIAR_ExGMAC);
            rtl8101_eri_write(baseAddr, 0x2E6, 2, 0x9003, ERIAR_ExGMAC);
            
            if (linuxData.mcfg == CFG_METHOD_17) {
                csiTmp = rtl8101_eri_read(baseAddr, 0x3FA, 2, ERIAR_ExGMAC);
                csiTmp |= BIT_14;
                rtl8101_eri_write(baseAddr, 0x3FA, 2, csiTmp, ERIAR_ExGMAC);
            }
            break;
    }
    
    switch (linuxData.mcfg) {
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            if (tp->org_pci_offset_99 & BIT_2)
                WriteReg8(0xB6, ReadReg8(0xB6) | BIT_0);
            break;
    }
    enablePCIOffset99();
}

/*
 * Corresponds to rtl8101_set_pci_99_180_exit_driver_para() in the Linux code.
 *
 * Updating the Linux code will most likely require this method
 * to be rewritten too.
 */
void RTL8100::setPCI99_180ExitDriverPara()
{
    switch (linuxData.mcfg) {
        case CFG_METHOD_15:
        case CFG_METHOD_16:
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            rtl8101_issue_offset_99_event(&linuxData);
            disablePCIOffset99();
            break;
    }
    
    switch (linuxData.mcfg) {
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            rtl8101_disable_pci_offset_180(&linuxData);
            break;
    }
}

/*
 * Corresponds to rtl8101_hw_d3_para() in the Linux code.
 *
 * Updating the Linux code will most likely require this method
 * to be rewritten too.
 */
void RTL8100::hardwareD3Para()
{
    struct rtl8101_private *tp = &linuxData;
    
    /* Set RxMaxSize register */
    WriteReg16(RxMaxSize, RX_BUF_SIZE);
    
    switch (tp->mcfg) {
        case CFG_METHOD_10:
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
        case CFG_METHOD_14:
        case CFG_METHOD_15:
        case CFG_METHOD_16:
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            WriteReg8(0xF1, ReadReg8(0xF1) & ~BIT_7);
            WriteReg8(Cfg9346, ReadReg8(Cfg9346) | Cfg9346_Unlock);
            WriteReg8(Config2, ReadReg8(Config2) & ~BIT_7);
            WriteReg8(Config5, ReadReg8(Config5) & ~BIT_0);
            WriteReg8(Cfg9346, ReadReg8(Cfg9346) & ~Cfg9346_Unlock);
            break;
    }
    if ((tp->mcfg == CFG_METHOD_11 || tp->mcfg == CFG_METHOD_12 ||
         tp->mcfg == CFG_METHOD_13) && (eeeAdv == 1))
        disableEEESupport();
    
    setPCI99_180ExitDriverPara();
    
    if (tp->mcfg == CFG_METHOD_17 || tp->mcfg == CFG_METHOD_18 ||
        tp->mcfg == CFG_METHOD_19) {
        /*disable ocp phy power saving*/
        spin_lock_irqsave(&tp->phy_lock, flags);
        mdio_write_phy_ocp(tp, 0x0C41, 0x13, 0x0000);
        mdio_write_phy_ocp(tp, 0x0C41, 0x13, 0x0500);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
    }
    
    if (tp->bios_setting & BIT_28) {
        if (tp->mcfg == CFG_METHOD_13) {
            if (!(ReadReg8(0xEF) & BIT_2)) {
                u32 gphy_val;
                
                spin_lock_irqsave(&tp->phy_lock, flags);
                mdio_write(tp, 0x1F, 0x0000);
                mdio_write(tp, 0x04, 0x0061);
                mdio_write(tp, 0x00, 0x1200);
                mdio_write(tp, 0x18, 0x0310);
                mdelay(20);
                mdio_write(tp, 0x1F, 0x0005);
                gphy_val = mdio_read(tp, 0x1a);
                gphy_val |= BIT_8 | BIT_0;
                mdio_write(tp, 0x1a, gphy_val);
                mdelay(30);
                mdio_write(tp, 0x1f, 0x0000);
                mdio_write(tp, 0x18, 0x8310);
                spin_unlock_irqrestore(&tp->phy_lock, flags);
            }
        }
    }
    rtl8101_disable_rxdvgate(&linuxData);
}

#define WAKE_ANY (WAKE_PHY | WAKE_MAGIC | WAKE_UCAST | WAKE_BCAST | WAKE_MCAST)


/*
 * Corresponds to rtl8101_powerdown_pll() in the Linux code.
 *
 * Updating the Linux code will most likely require this method
 * to be rewritten too.
 */
void RTL8100::powerdownPLL()
{
    struct rtl8101_private *tp = &linuxData;
    
    if (tp->wol_enabled == WOL_ENABLED) {
        int auto_nego;
        u16 anlpar_val;
        
        rtl8101_set_hw_wol(tp, tp->wol_opts);
        
        if (tp->mcfg == CFG_METHOD_17 || tp->mcfg == CFG_METHOD_18 ||
            tp->mcfg == CFG_METHOD_19) {
            WriteReg8(Cfg9346, ReadReg8(Cfg9346) | Cfg9346_Unlock);
            WriteReg8(Config2, ReadReg8(Config2) | PMSTS_En);
            WriteReg8(Cfg9346, ReadReg8(Cfg9346) & ~Cfg9346_Unlock);
        }
        mdio_write(tp, 0x1f, 0x0000);
        anlpar_val = mdio_read(tp, MII_LPA);
        
        if (tp->RequireResetPhyToChgSpd)
            rtl8101_hw_phy_config(tp);
        
        mdio_write(tp, 0x1f, 0x0000);
        auto_nego = mdio_read(tp, MII_ADVERTISE);
        auto_nego &= ~(ADVERTISE_10HALF | ADVERTISE_10FULL
                       | ADVERTISE_100HALF | ADVERTISE_100FULL);
        
        if (anlpar_val & (LPA_10HALF | LPA_10FULL))
            auto_nego |= (ADVERTISE_10HALF | ADVERTISE_10FULL);
        else
            auto_nego |= (ADVERTISE_100FULL | ADVERTISE_100HALF | ADVERTISE_10HALF | ADVERTISE_10FULL);
        
        mdio_write(tp, MII_ADVERTISE, auto_nego);
        mdio_write(tp, MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);
        
        switch (tp->mcfg) {
            case CFG_METHOD_1:
            case CFG_METHOD_2:
            case CFG_METHOD_3:
            case CFG_METHOD_4:
            case CFG_METHOD_5:
            case CFG_METHOD_6:
            case CFG_METHOD_7:
            case CFG_METHOD_8:
            case CFG_METHOD_9:
                break;
                
            default:
                WriteReg32(RxConfig, ReadReg32(RxConfig) | AcceptBroadcast | AcceptMulticast | AcceptMyPhys);
                break;
        }
        return;
    }
    rtl8101_phy_power_down(tp);
    
    switch (tp->mcfg) {
        case CFG_METHOD_6:
        case CFG_METHOD_9:
            WriteReg8(DBG_reg, ReadReg8(DBG_reg) | BIT_3);
            WriteReg8(PMCH, ReadReg8(PMCH) & ~BIT_7);
            break;
            
        case CFG_METHOD_8:
            pciDevice->extendedConfigWrite8(0x81, 0);
            WriteReg8(PMCH, ReadReg8(PMCH) & ~BIT_7);
            break;
            
        case CFG_METHOD_7:
        case CFG_METHOD_10:
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
        case CFG_METHOD_14:
        case CFG_METHOD_15:
        case CFG_METHOD_16:
        case CFG_METHOD_17:
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            WriteReg8(PMCH, ReadReg8(PMCH) & ~BIT_7);
            break;
            
        default:
            break;
    }
    
    switch (tp->mcfg) {
        case CFG_METHOD_18:
        case CFG_METHOD_19:
            WriteReg8(0xD0, ReadReg8(0xD0) & ~BIT_6);
            WriteReg8(0xF2, ReadReg8(0xF2) & ~BIT_6);
            break;
    }
}

#pragma mark --- RTL8100 timer action method ---

/*
 * This is the watchdog timer action routine. Its basic tasks are to:
 *  - check for link status changes and perform post link operations.
 *  - check for transmitter deadlocks.
 *  - update statistics and trigger statistics dumps.
 *
 * As early family members have a broken link change interrupt the
 * watchdog timer routine must check for link changes periodically.
 */

void RTL8100::timerActionRTL8100(IOTimerEventSource *timer)
{
    struct rtl8101_private *tp = &linuxData;
    UInt32 data32;
    UInt8 currLinkState;
    bool newLinkState;
    
    /* Check for link state events. */
    currLinkState = ReadReg8(PHYstatus);
    newLinkState = (currLinkState & LinkStatus) ? true : false;
    
    if (newLinkState != linkUp) {
        if (newLinkState) {
            /* Perform post link operations. */
            if (tp->mcfg == CFG_METHOD_5 || tp->mcfg == CFG_METHOD_6 ||
                tp->mcfg == CFG_METHOD_7 || tp->mcfg == CFG_METHOD_8)
                set_offset70F(tp, 0x3F);
            
            if (tp->mcfg == CFG_METHOD_11 || tp->mcfg == CFG_METHOD_12 ||
                tp->mcfg == CFG_METHOD_13) {
                if ((currLinkState & FullDup) == 0)
                    disableEEESupport();
                
                if (currLinkState & _10bps) {
                    rtl8101_eri_write(baseAddr, 0x1D0, 2, 0x4D02, ERIAR_ExGMAC);
                    rtl8101_eri_write(baseAddr, 0x1DC, 2, 0x0060, ERIAR_ExGMAC);
                    
                    rtl8101_eri_write(baseAddr, 0x1B0, 2, 0, ERIAR_ExGMAC);
                    mdio_write( tp, 0x1F, 0x0004);
                    data32 = mdio_read( tp, 0x10);
                    data32 |= 0x0400;
                    data32 &= ~0x0800;
                    mdio_write(tp, 0x10, data32);
                    mdio_write(tp, 0x1F, 0x0000);
                } else {
                    rtl8101_eri_write(baseAddr, 0x1D0, 2, 0, ERIAR_ExGMAC);
                    if (eeeAdv && (ReadReg8(0xEF) & BIT_0) == 0)
                        rtl8101_eri_write(baseAddr, 0x1B0, 2, 0xED03, ERIAR_ExGMAC);
                }
            } else if (tp->mcfg == CFG_METHOD_14 || tp->mcfg == CFG_METHOD_15 ||
                       tp->mcfg == CFG_METHOD_16) {
                if (currLinkState & _10bps) {
                    rtl8101_eri_write(baseAddr, 0x1D0, 2, 0x4d02, ERIAR_ExGMAC);
                    rtl8101_eri_write(baseAddr, 0x1DC, 2, 0x0060, ERIAR_ExGMAC);
                } else {
                    rtl8101_eri_write(baseAddr, 0x1D0, 2, 0, ERIAR_ExGMAC);
                }
            }
            
            if ((tp->mcfg == CFG_METHOD_17 || tp->mcfg == CFG_METHOD_18 ||
                 tp->mcfg == CFG_METHOD_19)) {
                if (currLinkState & FullDup)
                    WriteReg32(TxConfig, (ReadReg32(TxConfig) | (BIT_24 | BIT_25)) & ~BIT_19);
                else
                    WriteReg32(TxConfig, (ReadReg32(TxConfig) | BIT_25) & ~(BIT_19 | BIT_24));
                
                /*half mode*/
                if (!(currLinkState & FullDup)) {
                    spin_lock_irqsave(&tp->phy_lock, flags);
                    mdio_write(tp, 0x1F, 0x0000);
                    mdio_write(tp, MII_ADVERTISE, mdio_read(tp, MII_ADVERTISE)&~(ADVERTISE_PAUSE_CAP|ADVERTISE_PAUSE_ASYM));
                    spin_unlock_irqrestore(&tp->phy_lock, flags);
                }
            }
            setLinkUp(currLinkState);
        } else {
            setLinkDown();
            
            /* Perform post link operations. */
            if (tp->mcfg == CFG_METHOD_11 || tp->mcfg == CFG_METHOD_12 ||
                tp->mcfg == CFG_METHOD_13) {
                spin_lock_irqsave(&tp->phy_lock, flags);
                mdio_write( tp, 0x1F, 0x0004);
                data32 = mdio_read( tp, 0x10);
                data32 &= ~0x0C00;
                mdio_write(tp, 0x1F, 0x0000);
                spin_unlock_irqrestore(&tp->phy_lock, flags);
            }
            if (tp->mcfg == CFG_METHOD_5 || tp->mcfg == CFG_METHOD_6 ||
                tp->mcfg == CFG_METHOD_7 || tp->mcfg == CFG_METHOD_8)
                set_offset70F(tp, 0x17);
            
            switch (tp->mcfg) {
                case CFG_METHOD_17:
                    if (tp->org_pci_offset_99 & BIT_2)
                        tp->issue_offset_99_event = TRUE;
                    break;
            }
        }
    }
    /* Check for tx deadlock. */
    if (linkUp) {
        if (checkForDeadlock())
            goto done;
        
        updateStatitics();
    }
    /* We can savely free the mbuf here because the timer action gets called
     * synchronized to the workloop.
     */
    if (txNext2FreeMbuf) {
        freePacket(txNext2FreeMbuf);
        txNext2FreeMbuf = NULL;
    }
    
done:
    timerSource->setTimeoutMS(kTimeoutMS);
    txDescDoneLast = txDescDoneCount;
}

#pragma mark --- miscellaneous functions ---

static inline UInt32 adjustIPv6Header(mbuf_t m)
{
    struct ip6_hdr *ip6Hdr = (struct ip6_hdr *)((UInt8 *)mbuf_data(m) + ETHER_HDR_LEN);
    struct tcphdr *tcpHdr = (struct tcphdr *)((UInt8 *)ip6Hdr + sizeof(struct ip6_hdr));
    UInt32 plen = ntohs(ip6Hdr->ip6_ctlun.ip6_un1.ip6_un1_plen);
    UInt32 csum = ntohs(tcpHdr->th_sum) - plen;
    
    csum += (csum >> 16);
    ip6Hdr->ip6_ctlun.ip6_un1.ip6_un1_plen = 0;
    tcpHdr->th_sum = htons((UInt16)csum);
    
    return (plen + kMinL4HdrOffsetV6);
}

static unsigned const ethernet_polynomial = 0x04c11db7U;

static inline u32 ether_crc(int length, unsigned char *data)
{
    int crc = -1;
    
    while(--length >= 0) {
        unsigned char current_octet = *data++;
        int bit;
        for (bit = 0; bit < 8; bit++, current_octet >>= 1) {
            crc = (crc << 1) ^
            ((crc < 0) ^ (current_octet & 1) ? ethernet_polynomial : 0);
        }
    }
    return crc;
}

