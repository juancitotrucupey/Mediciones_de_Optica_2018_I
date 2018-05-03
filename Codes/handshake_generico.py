import serial
fout=open('T28.dat','w')
i=0
ser = serial.Serial('/dev/ttyACM0', 9600)
ser.flushOutput()
#ser.flushInput()

inbyte = ser.readline()
print(inbyte)
#inbyte = ser.read()
#inbyte = ser.read()
#inbyte = ser.read()
#inbyte = ser.read()
fout = open('T28.dat', 'a')
inbyte1=ser.readline()
D1=float(inbyte1)
#print (inbyte)
print(D1)
fout.write('{}\n'.format(D1))
fout.close()
#if (inbyte == b"A"):
 #   print ("el arduino dijo hola")
 
"""
i=0
Muestras=100 
while True:
     
     inbyte = ser.readline()
     referencia=float(inbyte)
     
     while referencia > -1000: 
         inbyte=ser.readline()
         referencia=float(inbyte)
 
         if (referencia == -10000):
         
             while i < Muestras:
                 
                 fout = open('T28.dat', 'a')
                 inbyte1=ser.readline()
                 inbyte2=ser.readline()
                 inbyte3=ser.readline()
                 inbyte4=ser.readline()
                 inbyte5=ser.readline()
                 inbyte6=ser.readline()
                 inbyte = ser.readline()
                 
                 D1=float(inbyte1)
                 D2=float(inbyte2)
                 D3=float(inbyte3)
                 D4=float(inbyte4)
                 D5=float(inbyte5)
                 D6=float(inbyte6)

  
                 print(i," ",D1," ",D2," ",D3," ",D4," ",D5," ",D6)

                 
                 fout.write('{} {} {} {} {} {}\n'.format(D1,D2,D3,D4,D5,D6))

                 fout.close()
                 i=i+1
             
             
         if (referencia == -1000000): 
              inbyte1=ser.readline()
              inbyte2=ser.readline()
              inbyte3=ser.readline()
              inbyte4=ser.readline()
              inbyte5=ser.readline()
              inbyte6=ser.readline()
              inbyte7=ser.readline()
              inbyte8=ser.readline()
              inbyte9=ser.readline()
              inbyte10=ser.readline()
              inbyte11=ser.readline()
              inbyte12=ser.readline()
              
              D1=float(inbyte1)
              D2=float(inbyte2)
              D3=float(inbyte3)
              D4=float(inbyte4)
              D5=float(inbyte5)
              D6=float(inbyte6)
              D7=float(inbyte7)
              D8=float(inbyte8)
              D9=float(inbyte9)
              D10=float(inbyte10)
              D11=float(inbyte11)
              D12=float(inbyte12)
              
              print ("R4",D1,"+/-",D2," ","R5",D3,"+/-",D4," ","R7",D5,"+/-",D6," ","T4",D7,"+/-",D8," ","T5",D9,"+/-",D10," ","T7",D11,"+/-",D12)
              print ("para terminar use solo ctrl + c")
  

"""
    
