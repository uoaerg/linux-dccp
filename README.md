# DCCP Test Tree

This is the test tree for the DCCP protocol. It contains patches that have not yet made it to the mainline kernel.
Some of them may be submitted to the networking development list in the future, others may be refactored.

## Sub-trees in this tree

All development is based on top of David S. Miller's current [net-tree](http://git.kernel.org/cgit/linux/kernel/git/davem/net.git/).

This is the basis for a stack of sub-trees:

| *Sub-tree* | Contents |
|------------|----------|
| `ready`    | Has extra patches that are either special (such as this file), or patches that are being submitted to the networking tree.|
| `dccp`     | General patches for DCCP and existing CCIDs. |
| `ccid4`    | An implementation of CCID4 [RFC5622](https://www.ietf.org/rfc/rfc5622.txt) by Leandro Sales de Melo, Ivo Calado, and Erivaldo Xavier.|
| `ccid5`    | A very interesting implementation of TCP-Cubic with Ack Vectors for DCCP by Ivo Calado and Leandro Sales de Melo. |


## Contributing

All questions, suggestions, and patches should be directed to [dccp@vger.kernel.org](mailto:dccp@vger.kernel.org "DCCP Development Mailing List").

## Links

* [Linux Foundation DCCP Page](http://www.linuxfoundation.org/collaborate/workgroups/networking/dccp)
* [Older DCCP Testing Notes](http://www.erg.abdn.ac.uk/users/gerrit/dccp/testing_dccp/)
* [DCCP Mailing List Archives](http://www.mail-archive.com/dccp@vger.kernel.org/maillist.html)