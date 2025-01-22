.. SPDX-License-Identifier: GPL-2.0-only or MIT

==========================================
Texas Instruments CPSW Proxy Client driver
==========================================

Introduction
============

The CPSW (Common Platform Switch) Ethernet Switch on TI's K3 SoCs provides
Ethernet functionality. There may be multiple instances of CPSW on a single
SoC. The term "CPSWnG" is used to indicate the number of MAC Ports supported
by a specific instance of CPSW. CPSWnG indicates that the peripheral has
(n-1) MAC Ports and 1 Host Port. Examples of existing instances are:
CPSW2G => 1 MAC Port and 1 Host Port
CPSW3G => 2 MAC Ports and 1 Host Port
CPSW5G => 4 MAC Ports and 1 Host Port
CPSW9G => 8 MAC Ports and 1 Host Port

The presence of 2 or more MAC Ports implies that Hardware Switching can
be enabled between the MAC Ports if required.

The "am65-cpsw-nuss.c" driver in Linux at:
drivers/net/ethernet/ti/am65-cpsw-nuss.c
provides Ethernet functionality for applications on Linux.
It also handles both the control-path and data-path, namely:
Control => Configuration of the CPSW Peripheral
Data => Configuration of the DMA Channels to transmit/receive data

The aforementioned configuration supports use-cases where all applications
which require Ethernet functionality are only running on Linux.

However, there are use-cases where applications running on different
Operating Systems across multiple cores on the SoC require Ethernet
functionality. Such use-cases can be supported by implementing a
Client-Server model to share the data-path among Clients while the Server
owns the control-path.

On TI's K3 SoCs (J721E, J7200 and J784S4 in particular), the Ethernet Switch
Firmware (EthFw) running on the MAIN R5F core acts as the Server and
configures the CPSWnG instance (CPSW5G on J7200 and CPSW9G on J721E, J784S4)
of the CPSW Ethernet Switch on the SoC. The Clients running on various cores
communicate with EthFw via RPMsg (Remote Processor Messaging) to request
resource allocation details during initialization, followed by requesting
EthFw to enable features supported by CPSW based on the features required
by the applications running on the respective cores.

EthFw handles requests from the Clients and evaluates them before configuring
CPSW based on the request. Since no Client is actually in control of CPSW and
only requests EthFw for configuring CPSW, EthFw acts as the proxy for the
Clients. Thus, the Linux Client which interfaces with EthFw is named:
CPSW Proxy Client

The data-path for the CPSW Proxy Client driver remains identical to the
"am65-cpsw-nuss.c" driver which happens to be DMA. It is only the control-path
that is different.

Client-Server discovery occurs over the RPMsg-Bus. EthFw announces its
RPMsg Endpoint name over the RPMsg-Bus. The CPSW Proxy Client driver
registers itself with the Linux RPMsg framework to be probed for the same
Endpoint name. Following probe, the Linux Client driver begins communicating
with EthFw and queries details of the resources available for the Linux Client.

Terminology
===========

Virtual Port
        A Virtual Port refers to the Software View of an Ethernet MAC Port.
        There are two types of Virtual Ports:
        1. Virtual MAC Only Port
        2. Virtual Switch Port

Virtual MAC Only Port
        A Virtual MAC only Port refers to a dedicated physical MAC Port for
        a Client. This corresponds to MAC Mode of operation in Ethernet
        Terminology. All traffic sent to or received from the Physical
        MAC Port is that of the Client to which the Virtual MAC Only Port
        has been allocated.

Virtual Switch Port
        A Virtual Switch Port refers to a group of physical MAC ports with
        Switching enabled across them. This implies that any traffic sent
        to the Port from a Client could potentially exit a Physical MAC
        Port along with the traffic from other Clients. Similarly, the traffic
        received on the Port by a Client could have potentially ingressed
        on a Physical MAC Port along with the traffic meant for other Clients.
        While the ALE (Address Lookup Engine) handles segregating the traffic,
        and the CPSW Ethernet Switch places traffic on dedicated RX DMA Flows
        meant for a single Client, it is worth noting that the bandwidths
        of the Physical MAC Port are shared by Clients when traffic is sent to
        or received from a Virtual Switch Port.

Network Interface
        The user-visible interface in Linux userspace exposed to applications
        that serves as the entry/exit point for traffic to/from the Virtual
        Ports. A single network interface (ethX) maps to either a Virtual
        MAC Only Port or a Virtual Switch Port.

C2S
        RPMsg source is Client and destination is Server.

S2C
        RPMsg source is Server and destination is Client.

Initialization Sequence
=======================

The sequence of message exchanges between the Client driver and EthFw starting
from the driver probe and ending with the interfaces being brought up is as
follows:
1. C2S ETHFW_VIRT_PORT_INFO requesting details of Virtual Ports available
   for the Linux Client.
2. S2C response containing requested details
3. C2S ETHFW_VIRT_PORT_ATTACH request for each Virtual Port allocated during
   step 2.
4. S2C response containing details of the MTU Size, number of Tx DMA Channels
   and RX DMA Flows for the specified Virtual Port. The *Features* associated
   with the Virtual Port are also shared such as Multicast Filtering capability.
5. C2S ETHFW_ALLOC_RX request for each RX DMA Flow for a Virtual Port.
6. S2C response containing details of the RX PSI-L Thread ID, Flow base and
   Flow offset.
7. C2S ETHFW_ALLOC_TX request for each TX DMA Channel for a Virtual Port.
8. S2C response containing details of the TX PSI-L Thread ID.
9. C2S ETHFW_ALLOC_MAC request for each Virtual Port.
10. S2C response containing the MAC Address corresponding to the Virtual Port.
11. C2S ETHFW_MAC_REGISTER request for each Virtual Port with the MAC Address
    allocated in step 10. This is necessary to steer packets that ingress on
    the MAC Ports of CPSW onto the RX DMA Flow for the Virtual Port in order
    to allow the Client to receive the packets.
12. S2C response indicating status of request.
13. C2S ETHFW_IPv4_REGISTER request *only* for Virtual Switch Port interface.
    The IPv4 address assigned to the "ethX" network interface in Linux
    corresponding to the Virtual Switch Port interface has to be registered
    with EthFw. This is due to the reason that all Broadcast requests including
    ARP requests received by the MAC Ports corresponding to the Virtual Switch
    Port are consumed solely be EthFw. Such traffic is sent to Clients by
    alternate methods. Therefore EthFw needs to know the IPv4 address for the
    "ethX" network interface in Linux in order to automatically respond to
    ARP requests, thereby enabling Unicast communication.
14. S2C response indicating status of request.
15. C2S ETHFW_MCAST_FILTER_ADD request to register the Multicast Addresses
    associated with the network interface corresponding to the Virtual Port
    which has the Multicast Filtering capability.
16. S2C response indicating status of request.
17. C2S ETHFW_MCAST_FILTER_DEL request to deregister the Multicast Addresses
    associated with the network interface corresponding to the Virtual Port
    which has the Multicast Filtering capability.
18. S2C response indicating status of request.

Shutdown Sequence
=================

The sequence of message exchanges between the Client driver and EthFw on module
removal are as follows:
1. C2S ETHFW_MAC_DEREGISTER request to deregister the MAC Address for each
   Virtual Port.
2. S2C response indicating status of request.
3. C2S ETHFW_MCAST_FILTER_DEL request to deregister the Multicast Addresses
   associated with the network interface corresponding to the Virtual Port
   which has the Multicast Filtering capability.
4. S2C response indicating status of request.
5. C2S ETHFW_FREE_MAC request to release the MAC Address allocated to each
   Virtual Port.
6. S2C response indicating status of request.
7. C2S ETHFW_FREE_TX request to release the TX DMA Channel for each TX Channel
   for every Virtual Port.
8. S2C response indicating status of request.
9. C2S ETHFW_FREE_RX request to release the RX DMA Flow for each RX Channel
   for every Virtual Port.
10. S2C response indicating status of request.
11. C2S ETHFW_VIRT_PORT_DETACH request to release each Virtual Port.
12. S2C response indicating status of request.

Features Supported
==================

The set of features supported in addition to providing basic Ethernet
Functionality are:
1. Multicast Filtering
2. Determining Link Status of the network interface corresponding to the
   Virtual MAC Only port via ethtool.
3. Interrupt Pacing/Coalescing
