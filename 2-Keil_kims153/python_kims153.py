import serial

#this is the serial code for UART to transmit into a text file
# and from the text file the data will be transmitted to excel for 3D modeling


s = serial.Serial("COM3", 115200) #using port 5 with baud rate of 115200

print("Opening: " + s.name)

s.write(b'8')           

while True: #infinite loop
    x = s.readline()        # read lines
    a = x.decode()      # convert byte type to str
    c = a.rstrip("\n\r") # strip \n \r to remove empty lines
    print(c)
    
print("Closing: " + s.name)
s.close();
