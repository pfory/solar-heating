--Init  
base = "/home/Corridor/esp07/"
deviceID = "ESP8266 Solar "..node.chipid()

heartBeat = node.bootreason()+10
print("Boot reason:"..heartBeat)

wifi.setmode(wifi.STATION)
wifi.sta.config("Datlovo","Nu6kMABmseYwbCoJ7LyG")
cfg={
  ip = "192.168.1.152",
  netmask = "255.255.255.0",
  gateway = "192.168.1.1"
}
wifi.sta.setip(cfg)
wifi.sta.autoconnect(1)

--Broker="88.146.202.186"  
Broker="192.168.1.56"  

sendDelay = 30000 --ms
received = ""

pinLed = 4
gpio.mode(pinLed,gpio.OUTPUT)  
gpio.write(pinLed,gpio.LOW)  
tmr.delay(1000000)
gpio.write(pinLed,gpio.HIGH)  



versionSW                  = "0.31"
versionSWString            = "Solar v" 
print(versionSWString .. versionSW)

function reconnect()
  print ("Waiting for Wifi")
  heartBeat = 20
  if wifi.sta.status() == 5 and wifi.sta.getip() ~= nil then 
    print ("Wifi Up!")
    tmr.stop(1) 
    m:connect(Broker, 1883, 0, 1, function(conn) 
      print(wifi.sta.getip())
      print("Mqtt Connected to:" .. Broker) 
      mqtt_sub() --run the subscription function 
    end)
  end
  heartBeat=20
  sendHB()
end

m = mqtt.Client(deviceID, 180, "datel", "hanka12")  
m:lwt("/lwt", deviceID, 0, 0)  
m:on("offline", function(con)   
  print ("Mqtt Reconnecting...")   
  tmr.alarm(1, 10000, 1, function()  
    reconnect()
  end)
end)  

uart.on("data", 0,
  function(data)
    --print("receive from uart:", data)
    received = received..data
end, 0)

function trim(s)
  return (s:gsub("^%s*(.-)%s*$", "%1"))
end

function sendData()
  gpio.write(pinLed,gpio.LOW)
  m:publish(base.."HeartBeat",              heartBeat,0,0)  
  m:publish(base.."VersionSWSolar",         versionSW,0,0)  
  if heartBeat==0 then heartBeat=1
  else heartBeat=0
  end
  objProp = {}
  prikaz = ""
  received=trim(received)
  --received = "#I;54.06#O;44.38#M;27.38#B;11.06#R;1$*"
  if trim(received)~="" then 
    print(received)
    if string.find(received,"*")~=nil then 
      index = 1
      for value in string.gmatch(received,"\#%w?\;%d*[\.%d]*") do 
        objProp [index] = value
        prikaz = string.sub(value, 2, 2)
        if prikaz == "I" then
         tIN = string.sub(value, 4, 99)
        end
        if prikaz == "O" then
         tOUT = string.sub(value, 4, 99)
        end
        if prikaz == "M" then
         tRoom = string.sub(value, 4, 99)
        end
        if prikaz == "B" then
         tBojler = string.sub(value, 4, 99)
        end

        if prikaz == "R" then
          --print(string.sub(value, 4, 4))
          if string.sub(value, 4, 4) == "1" then
            sPumpSolar = "ON"
          else
            sPumpSolar = "OFF"
          end
        end
        index = index + 1
      end
    end
  else 
    print("empty data")
    tIN=0
    tOUT=0
    tRoom=0
    tBojler=0
    sPumpSolar="OFF"
  end
  print(tIN)
  print(tOUT)
  print(sPumpSolar)
  print(tRoom)
  print(tBojler)
  print("I am sending data from Solar unit to OpenHab")
  received=""

  m:publish(base.."tIN",                   string.format("%.1f",tIN),0,0)  
  m:publish(base.."tOUT",                   string.format("%.1f",tOUT),0,0)  
  m:publish(base.."sPumpSolar",             sPumpSolar,0,0)  
  m:publish(base.."tRoom",                  string.format("%.1f",tRoom),0,0)  
  m:publish(base.."tBojler",                string.format("%.1f",tBojler),0,0)  
  gpio.write(pinLed,gpio.HIGH)  
end


function mqtt_sub()  
  m:subscribe(base.."com",0, function(conn)   
    print("Mqtt Subscribed to OpenHAB feed for device "..deviceID)  
  end)  
end

 -- on publish message receive event  
m:on("message", function(conn, topic, data)   
  print("Received:" .. topic .. ":" .. data) 
  if topic == base.."com" then
    if data == "ON" then
      print("Restarting ESP, bye.")
      node.restart()
    end
  end
end)  

function sendHB()
  print("I am sending HB to OpenHab")
  m:publish(base.."HeartBeat",   heartBeat,0,0)
 
  if heartBeat==0 then heartBeat=1
  else heartBeat=0
  end
end

tmr.alarm(0, 5000, 1, function() 
  parseData()
end)

uart.write(0,"Connecting to Wifi")
tmr.alarm(0, 1000, 1, function() 
  uart.write(0,".")
  if wifi.sta.status() == 5 and wifi.sta.getip() ~= nil then 
    print ("Wifi connected")
    tmr.stop(0) 
    m:connect(Broker, 1883, 0, 1, function(conn) 
      m:publish(base.."com",                    "OFF",0,0)  
      mqtt_sub() --run the subscription function 
      print(wifi.sta.getip())
      print("Mqtt Connected to:" .. Broker.." - "..base) 
      m:publish(base.."VersionSWSolar",         versionSW,0,0)  
      sendHB() 
      tmr.alarm(0, sendDelay, tmr.ALARM_AUTO, function()
        sendData() 
      end)
    end) 
  end
end)