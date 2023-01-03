
# Content

- [What is KSMBD?](#KSMBDwhat-is-ksmbd)
- [Under PFIF](#under-pfif)
- [Git](#git)
- [Maintainers](#maintainers)
- [Bug reports or contribution](#Bug-reports-or-contribution)
- [Features](#features)
- [Supported Linux Kernel Versions](#supported-linux-kernel-versions)
- [KSMBD architecture](#ksmbd-architecture)


## What is KSMBD?

KSMBD is an opensource In-kernel CIFS/SMB3 server created by Namjae Jeon for Linux Kernel. It's an implementation of SMB/CIFS protocol in kernel space for sharing files and IPC services over network. Initially the target is to provide improved file I/O performances, but the bigger goal is to have some new features which are much easier to develop and maintain inside the kernel and expose the layers fully. Directions can be attributed to sections where SAMBA is moving to few modules inside the kernel to have features like RDMA(Remote direct memory access) to work with actual performance gain.


## Under PFIF

This code was developed in participation with the Protocol Freedom Information Foundation.

Please see
* http://samba.org/samba/PFIF/
for more details.


## Git

The development git tree is available at
* https://github.com/cifsd-team/ksmbd
* https://github.com/cifsd-team/ksmbd-tools


## Maintainers

* Namjae Jeon <linkinjeon@kernel.org>


## Bug reports or contribution

For reporting bugs and sending patches, please send the patches to the following mail address:

* linkinjeon@kernel.org

or open issues/send PRs to [KSMBD](https://github.com/cifsd-team/ksmbd).

## Installing as a stand-alone module

Install prerequisite package for Fedora, RHEL:
```
	yum install kernel-devel-$(uname -r)
```

Build step:
```
	make
	sudo make install
```

To load the driver manually, run this as root:
```
	modprobe ksmbd
```


## Installing as a part of the kernel

1. Let's take [linux] as the path to your kernel source dir.
```
	cd [linux]
	cp -ar ksmbd [linux]/fs/
```

2. edit [linux]/fs/Kconfig
```
	source "fs/cifs/Kconfig"
	+source "fs/ksmbd/Kconfig"
	source "fs/coda/Kconfig"
```

3. edit [linux]/fs/Makefile
```
	obj-$(CONFIG_CIFS)              += cifs/
	+obj-$(CONFIG_SMB_SERVER)       += ksmbd/
	obj-$(CONFIG_HPFS_FS)           += hpfs/
```
4. make menuconfig and set ksmbd
```
	[*] Network File Systems  --->
		<M>   SMB server support
```

build your kernel


## Features

*Implemented*
1. SMB1(CIFS), SMB2/3 protocols for basic file sharing
2. Dynamic crediting
3. Compound requests
4. oplock/lease
5. Large MTU
6. NTLM/NTLMv2
7. Auto negotiation
8. HMAC-SHA256 Signing
9. Secure negotiate
10. Signing Update
11. Pre-authentication integrity(SMB 3.1.1)
12. SMB3 encryption(CCM, GCM)
13. SMB direct(RDMA)
14. Win-ACL
15. Kerberos
16. Multi-channel

*Planned*
1. Durable handle v1/v2
2. Persistent handles
3. Directory lease


## Supported Linux Kernel Versions

* Linux Kernel 5.4 or later


## KSMBD architecture

```
               |--- ...
       --------|--- ksmbd/3 - Client 3
       |-------|--- ksmbd/2 - Client 2
       |       |         _____________________________________________________
       |       |        |- Client 1                                           |
<--- Socket ---|--- ksmbd/1   <<= Authentication : NTLM/NTLM2, Kerberos(TODO)|
       |       |      | |      <<= SMB : SMB1, SMB2, SMB2.1, SMB3, SMB3.0.2,  |
       |       |      | |                SMB3.1.1                             |
       |       |      | |_____________________________________________________|
       |       |      |
       |       |      |--- VFS --- Local Filesystem
       |       |
KERNEL |--- ksmbd/0(forker kthread)
---------------||---------------------------------------------------------------
USER           ||
               || communication using NETLINK
               ||  ______________________________________________
               || |                                              |
        ksmbd.mountd <<= DCE/RPC, WINREG                         |
               ^  |  <<= configure shares setting, user accounts |
               |  |______________________________________________|
               |
               |------ smb.conf(config file)
               |
               |------ ksmbdpwd.db(user account/password file)
                            ^
  ksmbd.adduser ---------------|

```

## Performance

1. ksmbd vs samba performance comparison using iozone (Linux Client)
<br/><br/><img src="https://github.com/cifsd-team/cifsd-perf/blob/master/4k_read-write_performance.PNG"  width="850" height="450">

2. ksmbd vs samba performance comparison using fileop (Linux Client)
<br/><br/><img src="https://github.com/cifsd-team/cifsd-perf/blob/master/Fileop_throughput_Performance.PNG"  width="850" height="450">

3. ksmbd vs samba performance comparison using CrystalDiskMark (Windows Client)
<br/><br/>![CrystalDiskMark](https://github.com/cifsd-team/cifsd-perf/blob/master/CrystalDiskMark_Performance.JPG)
