--Init  
DeviceID="esp03"  
RoomID="bedNew"  

t = require('ds18b20')

-- ESP-01 GPIO Mapping
gpio0 = 3

t.setup(gpio0)
addrs = t.addrs()
if (addrs ~= nil) then
  print("Total DS18B20 sensors: "..table.getn(addrs))
end

wifi.setmode(wifi.STATION)
wifi.sta.config("Datlovo","Nu6kMABmseYwbCoJ7LyG")

Broker="213.192.58.66"  

function reconnect()
  print ("Waiting for Wifi")
  if wifi.sta.status() == 5 and wifi.sta.getip() ~= nil then 
    print ("Wifi Up!")
    tmr.stop(1) 
    m:connect(Broker, 31883, 0, function(conn) 
      print("Mqtt Connected to:" .. Broker) 
      mqtt_sub() --run the subscription function 
    end)
  end
end

m = mqtt.Client("ESP8266".. DeviceID, 180, "datel", "hanka")  
m:lwt("/lwt", "ESP8266", 0, 0)  
m:on("offline", function(con)   
  print ("Mqtt Reconnecting...")   
  tmr.alarm(1, 10000, 1, function()  
    reconnect()
  end)
end)

-- send data every 60 s
--tmr.alarm(2, 2000, 1, function() printTemp() end )

function printTemp()
  print("Temperature: "..t.read(addrs[1],C).."'C")
  print("Temperature: "..t.read(addrs[2],C).."'C")
  --t1=ds18b20.read()
end

function sendData()
  print("Sending temperatures")
  --print(addrs[1])
  m:publish("/home/".. RoomID .."/" .. DeviceID .. "/s0/state",string.format("%.3f", t.read(addrs[1])),0,0)  
  m:publish("/home/".. RoomID .."/" .. DeviceID .. "/s1/state",string.format("%.3f", t.read(addrs[2])),0,0)  
  m:publish("/home/".. RoomID .."/" .. DeviceID .. "/s2/state",string.format("%.3f", t.read(addrs[3])),0,0)  
  m:publish("/home/".. RoomID .."/" .. DeviceID .. "/s3/state",string.format("%.3f", t.read(addrs[1],'F')),0,0)  
  m:publish("/home/".. RoomID .."/" .. DeviceID .. "/s4/state",string.format("%.3f", t.read(addrs[2],'F')),0,0)  
  m:publish("/home/".. RoomID .."/" .. DeviceID .. "/s5/state",string.format("%.3f", t.read(addrs[3],'F')),0,0)  
  m:publish("/home/".. RoomID .."/" .. DeviceID .. "/s6/state",string.format("%.3f", t.read(addrs[1],'K')),0,0)  
  m:publish("/home/".. RoomID .."/" .. DeviceID .. "/s7/state",string.format("%.3f", t.read(addrs[2],'K')),0,0)  
end

tmr.alarm(0, 1000, 1, function() 
  print ("Connecting to Wifi... ")
  if wifi.sta.status() == 5 and wifi.sta.getip() ~= nil then 
    print ("Wifi connected")
    tmr.stop(0) 
    m:connect(Broker, 31883, 0, function(conn) 
      print("Mqtt Connected to:" .. Broker) 
      tmr.alarm(2, 10000, 1, function() sendData() end )
    end) 
  end
end)