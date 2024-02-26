"""
This script will send get request to lora cloud server and receive almanac bytestream.
It will then convert this byte stream into c array and write it to almanac.c file
"""
import requests
import os
import json
from base64 import decodebytes

URL = 'https://mgs.loracloud.com/api/v3/almanac/full'
#AUTH_TOKEN = 'AQEAdv9jq5dthzVHYZMRxDEg6WRUIaz590FXPuAfP+1jddncGODv'
AUTH_TOKEN = 'AQEAAP+vQhSLRLBeYzy5TZUhfV1scL5BJPy5aZDDqzdN2OjRxrIf'

response = requests.get(URL, headers = {'Ocp-Apim-Subscription-Key': AUTH_TOKEN})
print(response)


raw_almanac = response.json()['result']['almanac_image'].encode()
almanac_bin = decodebytes(raw_almanac)

# Path to lr11xx_almanac.h file
path_h = 'lr11xx_almanac.h'
path_c = 'lr11xx_almanac.c'

print(len(almanac_bin))
print(almanac_bin[0])
print(hex(almanac_bin[0]))
print(hex(almanac_bin[2579]))

response = response.json()
with open(path_c, "w") as f:
    f.write('/* AUTOGENERATED FILE - DO NOT MODIFY! */\n')
    f.write('#include "lr11xx_almanac.h"\n\n')
    f.write("lr11xx_gnss_almanac_full_read_bytestream_t lr11xx_full_almanac = {\n")
    for i in range(len(almanac_bin)):
        if i % 19 == 0 and i != 0:
            f.write(hex(almanac_bin[i]) + ",\n")
        else:
            f.write(hex(almanac_bin[i]) + ", ")
    f.write("\n};")
    f.write("\n\n")

with open(path_h, "w") as f:
    f.write('/* AUTOGENERATED FILE - DO NOT MODIFY! */\n')
    f.write('#ifndef LR11XX_ALMANAC_H\n')
    f.write('#define LR11XX_ALMANAC_H\n\n')
    f.write('#include <zephyr.h>\n')
    f.write('#include <lr11xx_gnss_types.h>\n\n')
    f.write("extern lr11xx_gnss_almanac_full_read_bytestream_t lr11xx_full_almanac;\n\n")
    f.write("#endif")


print("Almanac bytestream received, almanac.c file was created!")
