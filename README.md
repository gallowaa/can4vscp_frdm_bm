<h1> A baremetal port of VSCP firmware to Kinetis </h1>
<p> This is one of the pieces of the "EasyIoTwithVSCP" project on the Freescale Community. The first supported platform is the FRDM-K64F, however since the Kinetis SDK is used it is highly portable to other Kinetis devices with a FlexCAN peripheral. If you want to try this code you might want to use a TWR-K6x as the TWR-SER has a CAN transceiver. Alternatively, make a request for a FRDM-CAN-VSCP shield through the Freescale Community. </p>

<h2> Known Limitations </h2>
<p> Receiving CAN messages can cause a hardfault. This is being looked into and the problem does not exist with MQX, due to what I believe to be more robust transitioning between mailbox states (empty, full, overrun, etc) </p>
<p>  Comment out vscp_getEvent() in main.c to prevent hardfaults. The device will report vscp data (die temperature and accelerometer). </p>


<h2> Project Requirements</h2>

<h3> 1. Get the vscp firmware and software: </h3>
<ul>
    <li><code>git clone https://github.com/grodansparadis/vscp_software</code></li>
    <li><code>git clone https://github.com/grodansparadis/vscp_firmware</code></li>
</ul>

<h3> 2. Install the following free tools from Freescale: </h3>
<ul>
    <li> Freescale's new eclipse based IDE <a href="http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=KDS_IDE">Kinetis Design Studio 3.0.0</a></li>
    <li> Software Development Kit for Kinetis version 1.2 <a href="http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=KINETIS-SDK">Kinetis SDK 1.2</a></li>
</ul>

<h5> Setup the Freescale tools: </h5>
<ul>
<li> Accept the default KSDK installation path, ex. "C:/Freescale" on a Windows machine</li>
<li> An update is needed to make the IDE KSDK compatible. To install:</li>
<li><code> Help > Install New Software > Add > Archive > Browse C:/Freescale/KSDK_1.2.0/tools/eclipse_update/KSDK_1.2.0_Eclipse_Update.zip </code></li>
<li> When using the KSDK, we need to link to a platform library which contains hardware and operating system abstraction layers (HAL & OSA), peripheral drivers, and startup code. To Build the platform library: </li>
<li><code> File > Import > General > Existing Projects into Workspace > Next > Browse
C:/Freescale/KSDK_1.2.0/lib/ksdk_platform_lib/kds/<device_name></code></li>
<li> Don't forget to correct the include path for <code>vscp_firmware/common</code> to match your machine </li>
</ul>

<h3> 3. Additional Info </h3>
<ul>
<li> To install free debugger + serial emulator firmware on FRDM board 
<a href="http://mcuoneclipse.com/2014/04/27/segger-j-link-firmware-for-opensdav2/">MCU on Eclipse</a></li>
<li>Document for setting up a project based on the examples 
<a href="https://community.freescale.com/docs/DOC-103288">DOC-103288</a></li>
</ul>

![VSCP logo](http://vscp.org/images/vscp_logo.jpg)

<h2> Advantages of Kinetis for VSCP </h2>
<ul>
<li><b><i> FlexCAN hardware masking </i></b> - If there are 100 sensor nodes talking on the bus, there are many wasted CPU cycles involved in receiving messages through software only to find out that the message isn't for us. For example, a node is generally un-interested in seeing measurements (CLASS=10) from other nodes, but we DO want to know if the segment controller is trying to write to one of our registers (see vscp register model), or has told us to do something like go to sleep, change reporting interval, etc. This is easily accomplished with mailbox masking. </li>

<li><b><i> RTC Peripheral </i></b> - VSCP reference FW does limited time keeping through global variables, the Real-Time-Clock (RTC) module allows for high accuracy time keeping in various formats (tick time, POSIX time).  </li>
<li><b><i> Software Enablement </i></b> - KSDK decreases dependence on the reference manual, and MQX is a complimentary commercial grade RTOS that has been to space and comes with full set of drivers.
</ul>

<p> This is not meant to be an exhaustive list, just some things I noted during development and wanted to formalize. </p>


