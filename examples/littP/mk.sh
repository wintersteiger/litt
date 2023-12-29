# 5cc7c1fffedc5cae	Dining (Thermostat)
# 00124b00245c4c33	Bedroom Temp/Hum
# 00124b00239e6bc7	Guest Room Temp/Hum
# 00124b002517c1b6	Living Room Temp/Hum
# 00124b0029115603	Office Temp/Hum
# 001e5e0902901541  Central

# 18fc26000009c40a	Bedroom (Thermostat II)
# 70ac08fffe4dd48f	Office (Thermostat)
# 00158d00053d386b	Guest Room (Thermostat)
# 5cc7c1fffedc5cae Dining (Thermostat)
# f4ce36704c354b35	Bedroom (Thermostat III)

cmake -GNinja -DCMAKE_BUILD_TYPE=Debug -DWIFI_SSID=Camelopard -DWIFI_PASSWORD=Giraffe231 -DMQTT_BROKER_HOST=192.168.0.41 -DMQTT_CLIENT_ID=littP -DMQTT_USER=monitor -DMQTT_PASS=UirHdEAPDxiOCB5939VQ -DTEMPERATURE_TOPICS="Temperatures/001e5e0902901541|Central,Temperatures/5cc7c1fffedc5cae|Dining,Temperatures/00124b00245c4c33|Bedroom,Temperatures/00124b00239e6bc7|Guest Room,Temperatures/00124b0029115603|Office," -DDEMAND_TOPICS="Demands/5cc7c1fffedc5cae|Dining,Demands/f4ce36704c354b35|Bedroom,Demands/70ac08fffe4dd48f|Office,Demands/00158d00053d386b|Guest Room," ..
