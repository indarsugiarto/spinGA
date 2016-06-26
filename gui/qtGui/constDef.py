"""
SYNOPSYS
    Constant values for global definition.

Created on 2 Nov 2015
@author: indi
"""

#default port (usually)
SPIN3_HOST = '192.168.240.253'
SPIN5_HOST = '192.168.240.1'

CHIP_LIST_4  = [[0,0],[1,0],[0,1],[1,1]]
CHIP_LIST_48 = [[0,0],[1,0],[2,0],[3,0],[4,0],
                [0,1],[1,1],[2,1],[3,1],[4,1],[5,1],
                [0,2],[1,2],[2,2],[3,2],[4,2],[5,2],[6,2],
                [0,3],[1,3],[2,3],[3,3],[4,3],[5,3],[6,3],[7,3],
                      [1,4],[2,4],[3,4],[4,4],[5,4],[6,4],[7,4],
                            [2,5],[3,5],[4,5],[5,5],[6,5],[7,5],
                                  [3,6],[4,6],[5,6],[6,6],[7,6],
                                        [4,7],[5,7],[6,7],[7,7]]

#----------------------- DSP Related Stuffs ---------------------------
SEND_PORT = 17893 #tidak bisa diganti dengan yang lain
SEND_IPTAG = 0

#port for bidirect-communication
REPLY_PORT = 20000
REPLY_IPTAG = 2

#port for streaming data from SpiNNaker
RESULT_PORT = 20001  # previously: 17899
RESULT_IPTAG = 3

#another port with special coding structure
ERR_PORT = 20002
ERR_IPTAG = 4

SDP_PORT = 7 #Don't use Port 0 because it is reserved for debugging purpose
#Note, Port 2 will be used internally within the spiNNaker machine
SDP_CORE = 1
WITH_REPLY = 0x87
NO_REPLY = 0x07

######################### GUI parameters ########################
PEN_WIDTH = 2

#-------------------------- GA Stuffs ---------------------------
MAX_OBJVAL_SCALE = 100
MIN_OBJVAL_SCALE = 0

