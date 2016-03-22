base = "/home/Corridor/esp06/"
deviceID = "ESP8266 Solar "..node.chipid()

wifi.setmode(wifi.STATION)
wifi.sta.config("Datlovo","Nu6kMABmseYwbCoJ7LyG")

Broker="88.146.202.186"  

heartBeat     = node.bootreason()
received      = ""
tIN           = 0
tOUT          = 0
tPump         = 0
tBojler       = 0
sPumpStatus   = ""

function sendData()
  print("I am sending data to OpenHab")
  m:publish(base.."HeartBeat",    heartBeat,    0,0)
  m:publish(base.."tIN",          tIN,          0,0)
  m:publish(base.."tOUT",         tOUT,         0,0)
  m:publish(base.."tPump",        tPump,        0,0)
  m:publish(base.."tBojler",      tBojler,      0,0)
  m:publish(base.."sPumpStatus",  sPumpStatus,  0,0)
  if heartBeat==0 then heartBeat=1
  else heartBeat=0
  end
end


function parseData()
  objProp = {}
  prikaz = ""
  received = "#0;25.31#1;25.19#2;5.19#N;25.10#F;15.50#R;1#S;0#P;0.00#E;0.00#T0.00;#V;0.69#M;0#C;123456#A;0#W;12#O;1245$3600177622*"
  index = 1
  for value in string.gmatch(received,"\#%w?\;%d*[\.%d]*") do 
      objProp [index] = value
      prikaz = string.sub(value, 2, 2)
      if prikaz == "0" then
       tIN = string.sub(value, 4, 99)
      end
      if prikaz == "1" then
       tOUT = string.sub(value, 4, 99)
      end
      if prikaz == "2" then
       tBojler = string.sub(value, 4, 99)
      end
      if prikaz == "R" then
        if string.sub(value, 4, 4) == "1" then
          sPumpStatus = "ON"
        else
          sPumpStatus = "OFF"
        end
      end

      index = index + 1
  end
  sendData()
end

function reconnect()
  print ("Waiting for Wifi")
  if wifi.sta.status() == 5 and wifi.sta.getip() ~= nil then 
    print ("Wifi Up!")
    tmr.stop(1) 
    m:connect(Broker, 31883, 0, function(conn) 
      print(wifi.sta.getip())
      print("Mqtt Connected to:" .. Broker) 
      mqtt_sub() --run the subscription function 
    end)
  end
end

m = mqtt.Client(deviceID, 180, "datel", "hanka12")  
m:lwt("/lwt", deviceID, 0, 0)  
m:on("offline", function(con)   
  print("Mqtt Reconnecting...")   
  tmr.alarm(1, 10000, 1, function()  
    reconnect()
  end)
end)  


function mqtt_sub()  
  m:subscribe(base.."com",0, function(conn)   
    print("Mqtt Subscribed to OpenHAB feed for device "..deviceID)  
  end)  
end  

-- when 4 chars is received.
uart.on("data", 0,
  function(data)
    print("receive from uart:", data)
    received = data
    if data=="quit" then
      uart.on("data") -- unregister callback function
    end
end, 0)


tmr.alarm(0, 1000, 1, function() 
  print ("Connecting to Wifi... ")
  if wifi.sta.status() == 5 and wifi.sta.getip() ~= nil then 
    print ("Wifi connected")
    tmr.stop(0) 
    uart.setup(0, 9600, 8, uart.PARITY_NONE, uart.STOPBITS_1, 1)
    m:connect(Broker, 31883, 0, function(conn) 
      mqtt_sub() --run the subscription function 
      print(wifi.sta.getip())
      print("Mqtt Connected to:" .. Broker.." - "..base) 
      tmr.alarm(2, 10000, 1, function()  
        parseData()
      end)
    end) 
  end
end)