#import time
from networktables import NetworkTables

# To see messages from networktables, you must setup logging
import logging
logging.basicConfig(level=logging.DEBUG)

NetworkTables.initialize()
sd = NetworkTables.getTable("visiontable")
#sd.delete('visionTrigger')
while True:
    # print('dsTime:', sd.getNumber('dsTime', 'N/A'))
    x = 0
    y = 5
    print(sd.getNumber('angle',x))
    sd.putNumber('test',y)
