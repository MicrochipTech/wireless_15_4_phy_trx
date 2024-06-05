"""*****************************************************************************
* Copyright (C) 2024 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*****************************************************************************"""


Transceiver_Type_options      = ["RF212B","RF233", "RF215"]  


Pin_Configuration = {'IRQ':{'Pin':'',
                                    'Name':'IRQ',
                                    'Dir':'',
                                    'update':False},
                        'CLKM' :{'Pin':'',
                                    'Name':'CLKM',
                                    'Dir':'',
                                    'update':False},
                        'DIG1' :{'Pin':'',
                                    'Name':'DIG1',
                                    'Dir':'',
                                    'update':False},
                        'DIG2' :{'Pin':'',
                                    'Name':'DIG2',
                                    'Dir':'',
                                    'update':False},
                        'DIG3' :{'Pin':'',
                                    'Name':'DIG3',
                                    'Dir':'',
                                    'update':False},
                        'DIG4' :{'Pin':'',
                                    'Name':'DIG4',
                                    'Dir':'',
                                    'update':False},
                        '_RST' :{'Pin':'',
                                    'Name':'RST',
                                    'Dir':'',
                                    'update':False},   
                        'SLP_TR' :{'Pin':'',
                                    'Name':'SLP_TR',
                                    'Dir':'',
                                    'update':False},          
                        
                        }

global irq_pin_config_msg
irq_pin_config_msg = "**Set the pin functionality as *EIC_EXTINT4*, Pin name as *IRQ*, pin Direction as *In***"

global clkm_pin_config_msg
clkm_pin_config_msg = "**Set the pin functionality as *GPIO*, Pin name as *CLKM*, pin Direction as *In***"

global dig1_pin_config_msg
dig1_pin_config_msg = "**Set the pin functionality as *GPIO*, Pin name as *DIG1*, pin Direction as *In***"

global dig2_pin_config_msg
dig2_pin_config_msg = "**Set the pin functionality as *GPIO*, Pin name as *DIG2*, pin Direction as *In***"

global dig3_pin_config_msg
dig3_pin_config_msg = "**Set the pin functionality as *GPIO*, Pin name as *DIG3*, pin Direction as *In***"

global dig4_pin_config_msg
dig4_pin_config_msg = "**Set the pin functionality as *GPIO*, Pin name as *DIG4*, pin Direction as *In***"

global rst_pin_config_msg
rst_pin_config_msg = "**Set the pin functionality as *GPIO*, Pin name as *_RST*, pin Direction as *Out***"

global slp_tr_pin_config_msg
slp_tr_pin_config_msg = "**Set the pin functionality as *GPIO*, Pin name as *SLP_TR*, pin Direction as *Out***"
       
def handleMessage(messageID, args):
    print("Handle Message")


#######################################################################################
## GPIO Read
#######################################################################################
import re

def sort_alphanumeric(var):
    print("sort_alphanumeric")
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ]
    return sorted(var, key = alphanum_key)

def get_port_pin_dependencies():
    global pin_map_internal
    global portPackage
    pin_map_internal = {}
    package = {}
    global gpio_dependency
    gpio_dependency = []
    global gpio_attr_sym
    gpio_attr_sym = {}
    global gpio_attr_update
    gpio_attr_update = {}
    global pin_map
    pin_map = {}
    # Build package pinout map
    packageNode = ATDF.getNode("/avr-tools-device-file/variants")
    for id in range(0,len(packageNode.getChildren())):
        package[packageNode.getChildren()[id].getAttribute("package")] = packageNode.getChildren()[id].getAttribute("pinout")

    ## Find Number of unique pinouts
    uniquePinout = len(set(package.values()))
    ## get the pin count
    pincount = int(re.findall(r'\d+', package.keys()[0])[0])
    max_index = Database.getSymbolValue("core","PORT_PIN_MAX_INDEX")
    # Create portGroupName list of uppercase letters
    global portGroupName
    portGroupName = []
    for letter in range(65,91):
        portGroupName.append(chr(letter))
    
    portPackage =  Database.getSymbolValue("core","COMPONENT_PACKAGE")

    pinoutNode = ATDF.getNode('/avr-tools-device-file/pinouts/pinout@[name= "' + str(package.get(portPackage)) + '"]')
    for id in range(0,len(pinoutNode.getChildren())):
        if pinoutNode.getChildren()[id].getAttribute("type") == None:
            if "BGA" in portPackage or "WLCSP" in portPackage or "DRQFN" in portPackage:
                pin_map[pinoutNode.getChildren()[id].getAttribute("position")] = pinoutNode.getChildren()[id].getAttribute("pad")
            else:
                pin_map[int(pinoutNode.getChildren()[id].getAttribute("position"))] = pinoutNode.getChildren()[id].getAttribute("pad")
        else:
            pin_map_internal[pinoutNode.getChildren()[id].getAttribute("type").split("INTERNAL_")[1]] = pinoutNode.getChildren()[id].getAttribute("pad")

    if "BGA" in portPackage or "WLCSP" in portPackage or "DRQFN" in portPackage:
        pin_position = sort_alphanumeric(pin_map.keys())
        pin_position_internal = sort_alphanumeric(pin_map_internal.keys())
    else:
        pin_position = sorted(pin_map.keys())
        pin_position_internal = sorted(pin_map_internal.keys())

    internalPincount = pincount + len(pin_map_internal.keys())

    global pin_num_ID_map
    pin_num_ID_map = {}
    global pin_IDS
    pin_IDS = []
    for pinNumber in range(1, internalPincount + 1):
        gpio_local = {}
        if pinNumber < pincount + 1:
            gpio_attr_sym[str(pinNumber)] = ["core."+"PORT_PIN"+str(pinNumber),"core."+"PIN_" + str(pinNumber) + "_PORT_PIN", "core."+"PIN_" + str(pinNumber) + "_PORT_GROUP","core."+"PIN_" + str(pinNumber) + "_FUNCTION_NAME","core."+"PIN_" + str(pinNumber) + "_PERIPHERAL_FUNCTION","core."+"PIN_" + str(pinNumber) + "_INEN"]
            gpio_local['Pin_num'] = Database.getSymbolValue('core',"PORT_PIN"+str(pinNumber))
            gpio_local['Bit_Pos'] = Database.getSymbolValue('core',"PIN_" + str(pinNumber) + "_PORT_PIN")
            gpio_local['Group']   = Database.getSymbolValue('core',"PIN_" + str(pinNumber) + "_PORT_GROUP")
            gpio_local['Name'] = Database.getSymbolValue('core',"PIN_" + str(pinNumber) + "_FUNCTION_NAME")
            gpio_local['Pin_Func'] = Database.getSymbolValue('core',"PIN_" + str(pinNumber) + "_PERIPHERAL_FUNCTION")
            gpio_local['Dir'] = Database.getSymbolValue('core',"PIN_" + str(pinNumber) + "_DIR")
            gpio_local['INEN'] = Database.getSymbolValue('core',"PIN_" + str(pinNumber) + "_INEN")
            gpio_attr_update[str(pinNumber)] = gpio_local
            if str(gpio_attr_update[str(pinNumber)]['Bit_Pos']) != '0':
                pin_num_ID_map[pinNumber] = str('P'+str(gpio_attr_update[str(pinNumber)]['Group'])+ str(gpio_attr_update[str(pinNumber)]['Bit_Pos']))
                #pin_IDS.append(str('P'+str(gpio_attr_update[str(pinNumber)]['Group'])+ str(gpio_attr_update[str(pinNumber)]['Bit_Pos'])))
            for attr in range(len(gpio_attr_sym[str(pinNumber)])):
                gpio_dependency.append(gpio_attr_sym[str(pinNumber)][attr])
            check_Assigned_GPIO_Functionality(gpio_local['Name'],pinNumber)

        else:
            gpio_attr_sym[str(pinNumber)] = ["core."+"PORT_PIN"+str(pinNumber),"core."+"PIN_" + str(pinNumber) + "_PORT_PIN", "core."+"PIN_" + str(pinNumber) + "_PORT_GROUP","core."+"PIN_" + str(pinNumber) + "_FUNCTION_NAME","core."+"PIN_" + str(pinNumber) + "_PERIPHERAL_FUNCTION","core."+"PIN_" + str(pinNumber) + "_DIR","core."+"PIN_" + str(pinNumber) + "_INEN"]
            gpio_local['Pin_num'] = Database.getSymbolValue('core',"PORT_PIN"+str(pinNumber))
            gpio_local['Bit_Pos'] = Database.getSymbolValue('core',"PIN_" + str(pinNumber) + "_PORT_PIN")
            gpio_local['Group']   = Database.getSymbolValue('core',"PIN_" + str(pinNumber) + "_PORT_GROUP")
            gpio_local['Name'] = Database.getSymbolValue('core',"PIN_" + str(pinNumber) + "_FUNCTION_NAME")
            gpio_local['Pin_Func'] = Database.getSymbolValue('core',"PIN_" + str(pinNumber) + "_PERIPHERAL_FUNCTION")
            gpio_local['Dir'] = Database.getSymbolValue('core',"PIN_" + str(pinNumber) + "_DIR")
            gpio_local['INEN'] = Database.getSymbolValue('core',"PIN_" + str(pinNumber) + "_INEN")
            gpio_attr_update[str(pinNumber)] = gpio_local
            if str(gpio_attr_update[str(pinNumber)]['Bit_Pos']) != '0':
                pin_num_ID_map[pinNumber] = str('P'+str(gpio_attr_update[str(pinNumber)]['Group'])+ str(gpio_attr_update[str(pinNumber)]['Bit_Pos']))
                #pin_IDS.append(str('P'+str(gpio_attr_update[str(pinNumber)]['Group'])+ str(gpio_attr_update[str(pinNumber)]['Bit_Pos'])))
            for attr in range(len(gpio_attr_sym[str(pinNumber)])):
                gpio_dependency.append(gpio_attr_sym[str(pinNumber)][attr])
            check_Assigned_GPIO_Functionality(gpio_local['Name'],pinNumber)


def check_Assigned_GPIO_Functionality(GPIO_Name,pinNumber):
    print("check_Assigned_GPIO_Functionality")
    if GPIO_Name == 'IRQ':
        print("gpio name: IRQ, function type = ", gpio_local['Pin_Func'] )
        Pin_Configuration[str(gpio_attr_update[str(pinNumber)]['Name'])].update({'Pin': 'P'+str(gpio_attr_update[str(pinNumber)]['Group'])+ str(gpio_attr_update[str(pinNumber)]['Bit_Pos'])})
        Pin_Configuration[str('IRQ')].update({'update':True})
    if GPIO_Name == 'CLKM':
        print("gpio name: CLKM" )
        Pin_Configuration[str(gpio_attr_update[str(pinNumber)]['Name'])].update({'Pin': 'P'+str(gpio_attr_update[str(pinNumber)]['Group'])+ str(gpio_attr_update[str(pinNumber)]['Bit_Pos'])})
        Pin_Configuration[str('CLKM')].update({'update':True})  
    if GPIO_Name == 'DIG1':
        print("gpio name: DIG1" )
        Pin_Configuration[str(gpio_attr_update[str(pinNumber)]['Name'])].update({'Pin': 'P'+str(gpio_attr_update[str(pinNumber)]['Group'])+ str(gpio_attr_update[str(pinNumber)]['Bit_Pos'])})
        Pin_Configuration[str('DIG1')].update({'update':True})   
    if GPIO_Name == 'DIG2':
        print("gpio name: DIG2" )
        Pin_Configuration[str(gpio_attr_update[str(pinNumber)]['Name'])].update({'Pin': 'P'+str(gpio_attr_update[str(pinNumber)]['Group'])+ str(gpio_attr_update[str(pinNumber)]['Bit_Pos'])})
        Pin_Configuration[str('DIG2')].update({'update':True}) 
    if GPIO_Name == 'DIG3':
        print("gpio name: DIG3" )
        Pin_Configuration[str(gpio_attr_update[str(pinNumber)]['Name'])].update({'Pin': 'P'+str(gpio_attr_update[str(pinNumber)]['Group'])+ str(gpio_attr_update[str(pinNumber)]['Bit_Pos'])})
        Pin_Configuration[str('DIG3')].update({'update':True}) 
    if GPIO_Name == 'DIG4':
        print("gpio name: DIG4" )
        Pin_Configuration[str(gpio_attr_update[str(pinNumber)]['Name'])].update({'Pin': 'P'+str(gpio_attr_update[str(pinNumber)]['Group'])+ str(gpio_attr_update[str(pinNumber)]['Bit_Pos'])})
        Pin_Configuration[str('DIG4')].update({'update':True}) 
    if GPIO_Name == '_RST':
        print("gpio name: _RST" )
        Pin_Configuration[str(gpio_attr_update[str(pinNumber)]['Name'])].update({'Pin': 'P'+str(gpio_attr_update[str(pinNumber)]['Group'])+ str(gpio_attr_update[str(pinNumber)]['Bit_Pos'])})
        Pin_Configuration[str('_RST')].update({'update':True})
    if GPIO_Name == 'SLP_TR':
        print("gpio name: SLP_TR" )
        Pin_Configuration[str(gpio_attr_update[str(pinNumber)]['Name'])].update({'Pin': 'P'+str(gpio_attr_update[str(pinNumber)]['Group'])+ str(gpio_attr_update[str(pinNumber)]['Bit_Pos'])})
        Pin_Configuration[str('SLP_TR')].update({'update':True})


def GPIO_Update_Callback(symbol,event):
    print("GPIO_Update_Callback")
    print('GPIO callback Triggered')
    print("symbol:: ", symbol)
    print("event:: ", event)
    sym_ID = symbol.getID()
    print('sym_ID',sym_ID)

    print('event_val:',event['value'])

    pin_num = (re.findall(r'\d+', event['id']))[0]
    if 'PERIPHERAL_FUNCTION' in event['id']:
        print("gpio_attr_update:",gpio_attr_update[str(pin_num)] )
        gpio_attr_update[str(pin_num)].update({'Pin_Func' : str(event['value'])})
        print("gpio_attr_update after pin fn update:",gpio_attr_update[str(pin_num)] )
        print("pin configuration: ", Pin_Configuration["IRQ"])
        
        if 'P' + str(gpio_attr_update[str(pin_num)]['Group']) + str(gpio_attr_update[str(pin_num)]['Bit_Pos']) == Pin_Configuration["IRQ"]['Pin'] and gpio_attr_update[str(pin_num)]['Pin_Func'] != 'A':
            print("PA0==IRQ[pin] and pin function != EIC_EXTINT0")
            Pin_Configuration["IRQ"].update({'Pin':''})
            rfHostIrqPin.setValue('')

        if 'P' + str(gpio_attr_update[str(pin_num)]['Group']) + str(gpio_attr_update[str(pin_num)]['Bit_Pos']) == Pin_Configuration["CLKM"]['Pin'] and gpio_attr_update[str(pin_num)]['Pin_Func'] != 'GPIO':
            print("PA0==CLKM[pin] and pin function != GPIO")
            Pin_Configuration["CLKM"].update({'Pin':''})
            rfHostClkmPin.setValue('')

        if 'P' + str(gpio_attr_update[str(pin_num)]['Group']) + str(gpio_attr_update[str(pin_num)]['Bit_Pos']) == Pin_Configuration["DIG1"]['Pin'] and gpio_attr_update[str(pin_num)]['Pin_Func'] != 'GPIO':
            print("PA0==DIG1[pin] and pin function != GPIO")
            Pin_Configuration["DIG1"].update({'Pin':''})
            rfHostDig1Pin.setValue('')

        if 'P' + str(gpio_attr_update[str(pin_num)]['Group']) + str(gpio_attr_update[str(pin_num)]['Bit_Pos']) == Pin_Configuration["DIG2"]['Pin'] and gpio_attr_update[str(pin_num)]['Pin_Func'] != 'GPIO':
            print("PA0==DIG2[pin] and pin function != GPIO")
            Pin_Configuration["DIG2"].update({'Pin':''})
            rfHostDig2Pin.setValue('')

        if 'P' + str(gpio_attr_update[str(pin_num)]['Group']) + str(gpio_attr_update[str(pin_num)]['Bit_Pos']) == Pin_Configuration["DIG3"]['Pin'] and gpio_attr_update[str(pin_num)]['Pin_Func'] != 'GPIO':
            print("PA0==DIG3[pin] and pin function != GPIO")
            Pin_Configuration["DIG3"].update({'Pin':''})
            rfHostDig3Pin.setValue('')

        if 'P' + str(gpio_attr_update[str(pin_num)]['Group']) + str(gpio_attr_update[str(pin_num)]['Bit_Pos']) == Pin_Configuration["DIG4"]['Pin'] and gpio_attr_update[str(pin_num)]['Pin_Func'] != 'GPIO':
            print("PA0==DIG4[pin] and pin function != GPIO")
            Pin_Configuration["DIG4"].update({'Pin':''})
            rfHostDig4Pin.setValue('')

        if 'P' + str(gpio_attr_update[str(pin_num)]['Group']) + str(gpio_attr_update[str(pin_num)]['Bit_Pos']) == Pin_Configuration["_RST"]['Pin'] and gpio_attr_update[str(pin_num)]['Pin_Func'] != 'GPIO':
            print("PA0==_RST[pin] and pin function != GPIO")
            Pin_Configuration["_RST"].update({'Pin':''})
            rfHost_RstPin.setValue('')
    
        if 'P' + str(gpio_attr_update[str(pin_num)]['Group']) + str(gpio_attr_update[str(pin_num)]['Bit_Pos']) == Pin_Configuration["SLP_TR"]['Pin'] and gpio_attr_update[str(pin_num)]['Pin_Func'] != 'GPIO':
            print("PA0==SLP_TR[pin] and pin function != GPIO")
            Pin_Configuration["SLP_TR"].update({'Pin':''})
            rfHostSlpTrPin.setValue('')


    elif 'FUNCTION_NAME' in event['id']:
        gpio_attr_update[str(pin_num)].update({'Name' : str(event['value'])})
        if  str(event['value']) == 'IRQ' and gpio_attr_update[str(pin_num)]['Pin_Func'] == 'A':
            Pin_Configuration["IRQ"].update({'Pin':'P'+ str(gpio_attr_update[str(pin_num)]['Group'])+str(gpio_attr_update[str(pin_num)]['Bit_Pos'])})
            rfHostIrqPin.setValue(Pin_Configuration[str(event['value'])]['Pin'])

    
        if  str(event['value']) == 'CLKM' and gpio_attr_update[str(pin_num)]['Pin_Func'] == 'GPIO':
            Pin_Configuration["CLKM"].update({'Pin':'P'+ str(gpio_attr_update[str(pin_num)]['Group'])+str(gpio_attr_update[str(pin_num)]['Bit_Pos'])})
            rfHostClkmPin.setValue(Pin_Configuration[str(event['value'])]['Pin'])

        if  str(event['value']) == 'DIG1' and gpio_attr_update[str(pin_num)]['Pin_Func'] == 'GPIO':
            Pin_Configuration["DIG1"].update({'Pin':'P'+ str(gpio_attr_update[str(pin_num)]['Group'])+str(gpio_attr_update[str(pin_num)]['Bit_Pos'])})
            rfHostDig1Pin.setValue(Pin_Configuration[str(event['value'])]['Pin'])
            
        if  str(event['value']) == 'DIG2' and gpio_attr_update[str(pin_num)]['Pin_Func'] == 'GPIO':
            Pin_Configuration["DIG2"].update({'Pin':'P'+ str(gpio_attr_update[str(pin_num)]['Group'])+str(gpio_attr_update[str(pin_num)]['Bit_Pos'])})
            rfHostDig2Pin.setValue(Pin_Configuration[str(event['value'])]['Pin'])

        if  str(event['value']) == 'DIG3' and gpio_attr_update[str(pin_num)]['Pin_Func'] == 'GPIO':
            Pin_Configuration["DIG3"].update({'Pin':'P'+ str(gpio_attr_update[str(pin_num)]['Group'])+str(gpio_attr_update[str(pin_num)]['Bit_Pos'])})
            rfHostDig3Pin.setValue(Pin_Configuration[str(event['value'])]['Pin'])

        if  str(event['value']) == 'DIG4' and gpio_attr_update[str(pin_num)]['Pin_Func'] == 'GPIO':

            Pin_Configuration["DIG4"].update({'Pin':'P'+ str(gpio_attr_update[str(pin_num)]['Group'])+str(gpio_attr_update[str(pin_num)]['Bit_Pos'])})
            rfHostDig4Pin.setValue(Pin_Configuration[str(event['value'])]['Pin'])


        if  str(event['value']) == '_RST' and gpio_attr_update[str(pin_num)]['Pin_Func'] == 'GPIO':

            Pin_Configuration["_RST"].update({'Pin':'P'+ str(gpio_attr_update[str(pin_num)]['Group'])+str(gpio_attr_update[str(pin_num)]['Bit_Pos'])})
            rfHost_RstPin.setValue(Pin_Configuration[str(event['value'])]['Pin'])

        if  str(event['value']) == 'SLP_TR' and gpio_attr_update[str(pin_num)]['Pin_Func'] == 'GPIO':
            Pin_Configuration["SLP_TR"].update({'Pin':'P'+ str(gpio_attr_update[str(pin_num)]['Group'])+str(gpio_attr_update[str(pin_num)]['Bit_Pos'])})
            rfHostSlpTrPin.setValue(Pin_Configuration[str(event['value'])]['Pin'])

def check_gpio_dir():
    print("check_gpio_dir")
    reset_pin_flag = False
    status1_flag = False
    status2_flag = False

    if Pin_Configuration['IRQ']['Pin'] != '':
        reset_pin_dir = str(Database.getSymbolValue('core','PIN_'+ str(extract_pin_num_ID(pin_ID_req=str(Pin_Configuration['IRQ']['Pin']))) + "_DIR"))
        reset_pin_inen = str(Database.getSymbolValue('core','PIN_'+ str(extract_pin_num_ID(pin_ID_req=str(Pin_Configuration['IRQ']['Pin']))) + "_INEN"))
        if (reset_pin_dir == '') or (reset_pin_inen == 'True'):
            rnHostRstPinMsg.setLabel(rst_pin_dir_config_msg)
            rnHostRstPinMsg.setVisible(True)
            reset_pin_flag = True
    
    if Pin_Configuration['CLKM']['Pin'] != '':
        reset_pin_dir = str(Database.getSymbolValue('core','PIN_'+ str(extract_pin_num_ID(pin_ID_req=str(Pin_Configuration['CLKM']['Pin']))) + "_DIR"))
        reset_pin_inen = str(Database.getSymbolValue('core','PIN_'+ str(extract_pin_num_ID(pin_ID_req=str(Pin_Configuration['CLKM']['Pin']))) + "_INEN"))
        if (reset_pin_dir == '') or (reset_pin_inen == 'True'):
            rnHostRstPinMsg.setLabel(rst_pin_dir_config_msg)
            rnHostRstPinMsg.setVisible(True)
            reset_pin_flag = True

    if Pin_Configuration['DIG1']['Pin'] != '':
        reset_pin_dir = str(Database.getSymbolValue('core','PIN_'+ str(extract_pin_num_ID(pin_ID_req=str(Pin_Configuration['DIG1']['Pin']))) + "_DIR"))
        reset_pin_inen = str(Database.getSymbolValue('core','PIN_'+ str(extract_pin_num_ID(pin_ID_req=str(Pin_Configuration['DIG1']['Pin']))) + "_INEN"))
        if (reset_pin_dir == '') or (reset_pin_inen == 'True'):
            rnHostRstPinMsg.setLabel(rst_pin_dir_config_msg)
            rnHostRstPinMsg.setVisible(True)
            reset_pin_flag = True
        
    if Pin_Configuration['DIG2']['Pin'] != '':
        reset_pin_dir = str(Database.getSymbolValue('core','PIN_'+ str(extract_pin_num_ID(pin_ID_req=str(Pin_Configuration['DIG2']['Pin']))) + "_DIR"))
        reset_pin_inen = str(Database.getSymbolValue('core','PIN_'+ str(extract_pin_num_ID(pin_ID_req=str(Pin_Configuration['DIG2']['Pin']))) + "_INEN"))
        if (reset_pin_dir == '') or (reset_pin_inen == 'True'):
            rnHostRstPinMsg.setLabel(rst_pin_dir_config_msg)
            rnHostRstPinMsg.setVisible(True)
            reset_pin_flag = True

    if Pin_Configuration['DIG3']['Pin'] != '':
        reset_pin_dir = str(Database.getSymbolValue('core','PIN_'+ str(extract_pin_num_ID(pin_ID_req=str(Pin_Configuration['DIG3']['Pin']))) + "_DIR"))
        reset_pin_inen = str(Database.getSymbolValue('core','PIN_'+ str(extract_pin_num_ID(pin_ID_req=str(Pin_Configuration['DIG3']['Pin']))) + "_INEN"))
        if (reset_pin_dir == '') or (reset_pin_inen == 'True'):
            rnHostRstPinMsg.setLabel(rst_pin_dir_config_msg)
            rnHostRstPinMsg.setVisible(True)
            reset_pin_flag = True

    if Pin_Configuration['DIG4']['Pin'] != '':
        reset_pin_dir = str(Database.getSymbolValue('core','PIN_'+ str(extract_pin_num_ID(pin_ID_req=str(Pin_Configuration['DIG4']['Pin']))) + "_DIR"))
        reset_pin_inen = str(Database.getSymbolValue('core','PIN_'+ str(extract_pin_num_ID(pin_ID_req=str(Pin_Configuration['DIG4']['Pin']))) + "_INEN"))
        if (reset_pin_dir == '') or (reset_pin_inen == 'True'):
            rnHostRstPinMsg.setLabel(rst_pin_dir_config_msg)
            rnHostRstPinMsg.setVisible(True)
            reset_pin_flag = True

    if Pin_Configuration['_RST']['Pin'] != '':
        reset_pin_dir = str(Database.getSymbolValue('core','PIN_'+ str(extract_pin_num_ID(pin_ID_req=str(Pin_Configuration['_RST']['Pin']))) + "_DIR"))
        reset_pin_inen = str(Database.getSymbolValue('core','PIN_'+ str(extract_pin_num_ID(pin_ID_req=str(Pin_Configuration['_RST']['Pin']))) + "_INEN"))
        if (reset_pin_dir == '') or (reset_pin_inen == 'True'):
            rnHostRstPinMsg.setLabel(rst_pin_dir_config_msg)
            rnHostRstPinMsg.setVisible(True)
            reset_pin_flag = True

    if Pin_Configuration['SLP_TR']['Pin'] != '':
        reset_pin_dir = str(Database.getSymbolValue('core','PIN_'+ str(extract_pin_num_ID(pin_ID_req=str(Pin_Configuration['SLP_TR']['Pin']))) + "_DIR"))
        reset_pin_inen = str(Database.getSymbolValue('core','PIN_'+ str(extract_pin_num_ID(pin_ID_req=str(Pin_Configuration['SLP_TR']['Pin']))) + "_INEN"))
        if (reset_pin_dir == '') or (reset_pin_inen == 'True'):
            rnHostRstPinMsg.setLabel(rst_pin_dir_config_msg)
            rnHostRstPinMsg.setVisible(True)
            reset_pin_flag = True


def check_update_pins():
    if Pin_Configuration['IRQ']['update'] == True:
        rfHostIrqPin.setValue(str(Pin_Configuration['IRQ']['Pin']))
        Pin_Configuration[str('IRQ')].update({'update':False})

    if Pin_Configuration['CLKM']['update'] == True:
        rfHostClkmPin.setValue(str(Pin_Configuration['CLKM']['Pin']))
        Pin_Configuration[str('CLKM')].update({'update':False})

    if Pin_Configuration['DIG1']['update'] == True:
        rfHostDig1Pin.setValue(str(Pin_Configuration['DIG1']['Pin']))
        Pin_Configuration[str('DIG1')].update({'update':False})

    if Pin_Configuration['DIG2']['update'] == True:
        rfHostDig2Pin.setValue(str(Pin_Configuration['DIG2']['Pin']))
        Pin_Configuration[str('DIG2')].update({'update':False})

    if Pin_Configuration['DIG3']['update'] == True:
        rfHostDig3Pin.setValue(str(Pin_Configuration['DIG3']['Pin']))
        Pin_Configuration[str('DIG3')].update({'update':False})

    if Pin_Configuration['DIG4']['update'] == True:
        rfHostDig4Pin.setValue(str(Pin_Configuration['DIG4']['Pin']))
        Pin_Configuration[str('DIG4')].update({'update':False})
    
    if Pin_Configuration['_RST']['update'] == True:
        rfHost_RstPin.setValue(str(Pin_Configuration['_RST']['Pin']))
        Pin_Configuration[str('_RST')].update({'update':False})

    if Pin_Configuration['SLP_TR']['update'] == True:
        rfHostSlpTrPin.setValue(str(Pin_Configuration['SLP_TR']['Pin']))
        Pin_Configuration[str('SLP_TR')].update({'update':False})

    #check_gpio_dir()

def extract_pin_num_ID(pin_num_req = '',pin_ID_req = '',req = 'PIN_NUM'):
    print("extract_pin_num_ID")
    for pin_num, pin_ID in pin_num_ID_map.items():
        if req ==  'PIN_NUM':
            if pin_ID_req == pin_ID:
                return pin_num
        elif req == 'PIN_ID':
            if pin_num_req == pin_num:
                return pin_ID
            


        

######################################################################################

#                               Module Dependencies Harmony Callback
#########################################################################################################

def trxMethodUpdate(symbol, event):
    Transceiver_type = event["value"]
    file_list_RF233 = [RF233_phy_tx_frame_done_cbC, RF233_phy_taskC, RF233_phy_rx_frame_cbC, RF233_phy_ed_end_cbC, RF233_tfaC, RF233_phy_txC, RF233_phy_rx_enableC, RF233_phy_rxC, RF233_phy_pwr_mgmtC, RF233_phy_pibC, RF233_phy_irq_handlerC, RF233_phy_initC, RF233_phy_helperC, RF233_phy_edC, RF233_phyC, at86rf233H, RF233_phy_txH, RF233_phy_trx_reg_accessC, RF233_phy_trx_reg_accessH, RF233_phy_rxH, RF233_phy_pibH, RF233_phy_irq_handlerH, RF233_phy_internalH, RF233_phy_tasksH, RF233_phy_constantsH, RF233_phy_configH, RF233_ieee_phy_constH, RF233_phyH]
    file_list_RF212b = [RF212b_phy_tx_frame_done_cbC, RF212b_phy_taskC, RF212b_phy_rx_frame_cbC, RF212b_phy_ed_end_cbC, RF212b_tfaC, RF212b_phy_txC, RF212b_phy_rx_enableC, RF212b_phy_rxC, RF212b_phy_pwr_mgmtC, RF212b_phy_pibC, RF212b_phy_irq_handlerC, RF212b_phy_initC, RF212b_phy_helperC, RF212b_phy_edC, RF212b_phyC, at86rf212bH, RF212b_phy_txH, RF212b_phy_trx_reg_accessC, RF212b_phy_trx_reg_accessH, RF212b_phy_rxH, RF212b_phy_pibH, RF212b_phy_irq_handlerH, RF212b_phy_internalH, RF212b_phy_tasksH, RF212b_phy_constantsH, RF212b_phy_configH, RF212b_ieee_phy_constH, RF212b_phyH]
    if Transceiver_type == "RF233":
        print("transceiver = RF233")
        print("condTrxTypeRF215",condTrxTypeRF215)
        # symbol.setEnabled(True)
        rfHostlibSelectTRX.setValue("RF233")
        for file in file_list_RF212b:
            file.setEnabled(False)
        for file in file_list_RF215:
            file.setEnabled(False)
        for file in file_list_RF233:
            file.setEnabled(True)
        # for file in range(len(file_list_RF233)):
        #     file_list_RF212b[file].setEnabled(False)
        #     file_list_RF215[file].setEnabled(False)
        #     file_list_RF233[file].setEnabled(True)
        preprocessorMacro = preprocessorCompiler.getValue()
        preprocessorMacro = preprocessorMacro.replace(";PHY_AT86RF212B", ";PHY_AT86RF233")
        preprocessorMacro = preprocessorMacro.replace(";RF215V3", ";PHY_AT86RF233")
        preprocessorCompiler.setValue(preprocessorMacro)
        preprocessorCompiler.setEnabled(True)
    elif Transceiver_type == "RF212B":
        print("transceiver = RF212b")
        print("condTrxTypeRF215",condTrxTypeRF215)
        # symbol.setEnabled(True) #Neha
        rfHostlibSelectTRX.setValue("RF212B")
        for file in file_list_RF233:
            file.setEnabled(False)
        for file in file_list_RF215:
            file.setEnabled(False)
        for file in file_list_RF212b:
            file.setEnabled(True)
        # for file in range(len(file_list_RF212b)):
        #     file_list_RF233[file].setEnabled(False)
        #     file_list_RF215[file].setEnabled(False)
        #     file_list_RF212b[file].setEnabled(True)
        preprocessorMacro = preprocessorCompiler.getValue()
        preprocessorMacro = preprocessorMacro.replace(";PHY_AT86RF233", ";PHY_AT86RF212B")
        preprocessorMacro = preprocessorMacro.replace(";RF215V3", ";PHY_AT86RF212B")
        preprocessorCompiler.setValue(preprocessorMacro)
        preprocessorCompiler.setEnabled(True)
    elif Transceiver_type == "RF215":
        print("transceiver = RF215")
        for file in file_list_RF233:
            file.setEnabled(False)
        for file in file_list_RF212b:
            file.setEnabled(False)
        for file in file_list_RF215:
            file.setEnabled(True)
        # symbol.setEnabled(True)
        rfHostlibSelectTRX.setValue("RF215")
        condTrxRF215 = True
        print("condTrxTypeRF215",condTrxTypeRF215)
        
        print("rfHostlibSelectTRX.getValue()",rfHostlibSelectTRX.getValue())
        print("len of file_list_RF215[file]",len(file_list_RF215))
        # condTrxRF215 = (rfHostlibSelectTRX.getValue() == "RF215")
        # print("condTrxRF215",condTrxRF215)
        # condTrxTypeRF215 = [condTrxRF215, trxMethodUpdate, ['RF_HOST_SELECT_TRANSEIVER']]
        # print("condTrxTypeRF215",condTrxTypeRF215)
        preprocessorMacro = preprocessorCompiler.getValue()
        preprocessorMacro = preprocessorMacro.replace(";PHY_AT86RF233", ";RF215V3;SUPPORT_LEGACY_OQPSK;SUPPORT_OQPSK;SUPPORT_OFDM;SUPPORT_FSK;MULTI_TRX_SUPPORT;ENABLE_TFA;SUPPORT_MODE_SWITCH;TFA_CW;PROMISCUOUS_MODE;ENABLE_TFA; TFA_CW;")
        preprocessorMacro = preprocessorMacro.replace(";PHY_AT86RF212B", ";RF215V3;SUPPORT_LEGACY_OQPSK;SUPPORT_OQPSK;SUPPORT_OFDM;SUPPORT_FSK;MULTI_TRX_SUPPORT;ENABLE_TFA;SUPPORT_MODE_SWITCH;TFA_CW;PROMISCUOUS_MODE;ENABLE_TFA;TFA_CW;")
        preprocessorCompiler.setValue(preprocessorMacro)
        preprocessorCompiler.setEnabled(True)
    else:
        #to be implemented
        pass

def onAttachmentConnected(source, target):
    localComponent = source["component"]
    remoteComponent = target["component"]
    remoteID = remoteComponent.getID()
    connectID = source["id"]
    targetID = target["id"]  
    if "SERCOM" in targetID:
        rfHostlibSpiSercomType.setValue(targetID)

    
def onAttachmentDisconnected(source, target):
    print("SERCOM Attachment Disconnected")


def eic_used_callback(synbol, event):
    channel_num = event["id"][9:]
    if (event["value"]==True):
        rfHostlibEicChannelSelected.setValue(channel_num)
    elif (event["value"]==False):
            rfHostlibEicChannelSelected.setValue("")



#####################################################################################
#                            Component Generic Harmony API's 
#####################################################################################

def instantiateComponent(rfHostLib):
    Log.writeInfoMessage("Initiating RF HOST LIBRARY")
    configName = Variables.get("__CONFIGURATION_NAME")
    processor = Variables.get("__PROCESSOR")
    print("processor", processor)
    global file_list_RF215 
    file_list_RF215 = []
    requiredComponents = [
    "HarmonyCore",
    "sys_time",
    "RTOS",
    "eic",
    "trng",
    ]
    global srcFileSym
    global incFileSym
    res = Database.activateComponents(requiredComponents)

    global rfHostlibSpiSercomType
    rfHostlibSpiSercomType = rfHostLib.createStringSymbol("SELECTED_SERCOM", None)
    rfHostlibSpiSercomType.setLabel("Selected sercom")
    rfHostlibSpiSercomType.setDescription("Selected sercom type")
    rfHostlibSpiSercomType.setVisible(False)
   

    extIntNode = ATDF.getNode("/avr-tools-device-file/devices/device/peripherals/module@[name=\"EIC\"]/instance@[name=\"EIC\"]/parameters/param@[name=\"EXTINT_NUM\"]")
    if (extIntNode is None):
        extIntNode = ATDF.getNode("/avr-tools-device-file/devices/device/peripherals/module@[name=\"EIC\"]/instance@[name=\"EIC\"]/parameters/param@[name=\"CHIP_EIC_EXTINT_NUM\"]")
    extIntCount = int(extIntNode.getAttribute("value"))
    list_of_EicChannel_ID = []
    for extIntIndex in range(0 , extIntCount):
        list_of_EicChannel_ID.append("eic.EIC_CHAN_" + str(extIntIndex))


    # transceiver selected
    global rfHostlibEicChannelSelected
    rfHostlibEicChannelSelected = rfHostLib.createStringSymbol("SELECTED_EIC_CHANNEL", None)
    rfHostlibEicChannelSelected.setLabel("Eic channel selected")
    rfHostlibEicChannelSelected.setDescription("Selected eic channel")
    rfHostlibEicChannelSelected.setVisible(False)
    rfHostlibEicChannelSelected.setDependencies(eic_used_callback, list_of_EicChannel_ID)

    #rfHostlibSelectTransceiverType
    global rfHostlibSelectTRX
    rfHostlibSelectTRX = rfHostLib.createComboSymbol("RF_HOST_SELECT_TRANSEIVER", None, Transceiver_Type_options)
    rfHostlibSelectTRX.setLabel("Select Transceiver Type")
    rfHostlibSelectTRX.setDescription("Select the TRANSEIVER type")
    rfHostlibSelectTRX.setDefaultValue("RF233")
    rfHostlibSelectTRX.setDependencies(trxMethodUpdate, ["RF_HOST_SELECT_TRANSEIVER"])
        
    global condTrxRF215
    global condTrxTypeRF215
    condTrxRF215 = (rfHostlibSelectTRX.getValue()=="RF215")
    print("rfHostlibSelectTRX.getValue()",rfHostlibSelectTRX.getValue())
    #condTrxTypeRF215 = [condTrxRF215, trxMethodUpdate, ['RF_HOST_SELECT_TRANSEIVER']]
    condTrxTypeRF215 = [False, None, []]
    print("condTrxTypeRF215",condTrxTypeRF215)
    ## GPIO PORT DEPENDENCIES 
    get_port_pin_dependencies()

    global rnHostPinConfigmenu
    rnHostPinConfigmenu  = rfHostLib.createMenuSymbol("RN_HOST_LIB_PIN_MENU",None)
    rnHostPinConfigmenu.setLabel("Module Pin Selections")
    rnHostPinConfigmenu.setDescription("Select the required Pin configuration")


    global rfHostIrqPin
    rfHostIrqPin = rfHostLib.createStringSymbol("RST_PIN_UPDATE",rnHostPinConfigmenu)
    rfHostIrqPin.setLabel("Interrupt request pin")
    rfHostIrqPin.setReadOnly(True)
    rfHostIrqPin.setDependencies(GPIO_Update_Callback,gpio_dependency)

    global rfHostClkmPin
    rfHostClkmPin = rfHostLib.createStringSymbol("CLKM_PIN_UPDATE",rnHostPinConfigmenu)
    rfHostClkmPin.setLabel("Clock pin")
    rfHostClkmPin.setReadOnly(True)
    rfHostClkmPin.setDependencies(GPIO_Update_Callback,gpio_dependency)

    global rfHostDig1Pin
    rfHostDig1Pin = rfHostLib.createStringSymbol("DIG1_PIN_UPDATE",rnHostPinConfigmenu)
    rfHostDig1Pin.setLabel("DIG1 pin")
    rfHostDig1Pin.setReadOnly(True)
    rfHostDig1Pin.setDependencies(GPIO_Update_Callback,gpio_dependency)

    global rfHostDig2Pin
    rfHostDig2Pin = rfHostLib.createStringSymbol("DIG2_PIN_UPDATE",rnHostPinConfigmenu)
    rfHostDig2Pin.setLabel("DIG2 pin")
    rfHostDig2Pin.setReadOnly(True)
    rfHostDig2Pin.setDependencies(GPIO_Update_Callback,gpio_dependency)

    global rfHostDig3Pin
    rfHostDig3Pin = rfHostLib.createStringSymbol("DIG3_PIN_UPDATE",rnHostPinConfigmenu)
    rfHostDig3Pin.setLabel("DIG3 pin")
    rfHostDig3Pin.setReadOnly(True)
    rfHostDig3Pin.setDependencies(GPIO_Update_Callback,gpio_dependency)

    global rfHostDig4Pin
    rfHostDig4Pin = rfHostLib.createStringSymbol("DIG4_PIN_UPDATE",rnHostPinConfigmenu)
    rfHostDig4Pin.setLabel("DIG4 pin")
    rfHostDig4Pin.setReadOnly(True)
    rfHostDig4Pin.setDependencies(GPIO_Update_Callback,gpio_dependency)

    global rfHost_RstPin
    rfHost_RstPin = rfHostLib.createStringSymbol("_RST_PIN_UPDATE",rnHostPinConfigmenu)
    rfHost_RstPin.setLabel("Reset pin")
    rfHost_RstPin.setReadOnly(True)
    rfHost_RstPin.setDependencies(GPIO_Update_Callback,gpio_dependency)

    global rfHostSlpTrPin
    rfHostSlpTrPin = rfHostLib.createStringSymbol("SLP_TR_PIN_UPDATE",rnHostPinConfigmenu)
    rfHostSlpTrPin.setLabel("Sleep/Transmit_Receive")
    rfHostSlpTrPin.setReadOnly(True)
    rfHostSlpTrPin.setDependencies(GPIO_Update_Callback,gpio_dependency)


    global rfHostIrqPinMsg
    rfHostIrqPinMsg = rfHostLib.createCommentSymbol(None,rfHostIrqPin)
    rfHostIrqPinMsg.setVisible(True)
    rfHostIrqPinMsg.setLabel(irq_pin_config_msg)

    global rfHostClkmPinMsg
    rfHostClkmPinMsg = rfHostLib.createCommentSymbol(None,rfHostClkmPin)
    rfHostClkmPinMsg.setVisible(True)
    rfHostClkmPinMsg.setLabel(clkm_pin_config_msg)

    global rfHostDig1PinMsg
    rfHostDig1PinMsg = rfHostLib.createCommentSymbol(None,rfHostDig1Pin)
    rfHostDig1PinMsg.setVisible(True)
    rfHostDig1PinMsg.setLabel(dig1_pin_config_msg)

    global rfHostDig2PinMsg
    rfHostDig2PinMsg = rfHostLib.createCommentSymbol(None,rfHostDig2Pin)
    rfHostDig2PinMsg.setVisible(True)
    rfHostDig2PinMsg.setLabel(dig2_pin_config_msg)

    global rfHostDig3PinMsg
    rfHostDig3PinMsg = rfHostLib.createCommentSymbol(None,rfHostDig3Pin)
    rfHostDig3PinMsg.setVisible(True)
    rfHostDig3PinMsg.setLabel(dig3_pin_config_msg)

    global rfHostDig4PinMsg
    rfHostDig4PinMsg = rfHostLib.createCommentSymbol(None,rfHostDig4Pin)
    rfHostDig4PinMsg.setVisible(True)
    rfHostDig4PinMsg.setLabel(dig4_pin_config_msg)

    global rfHostSlptrPinMsg
    rfHostSlptrPinMsg = rfHostLib.createCommentSymbol(None,rfHostSlpTrPin)
    rfHostSlptrPinMsg.setVisible(True)
    rfHostSlptrPinMsg.setLabel(slp_tr_pin_config_msg)

    global rfHostRstPinMsg
    rfHostRstPinMsg = rfHostLib.createCommentSymbol(None,rfHost_RstPin)
    rfHostRstPinMsg.setVisible(True)
    rfHostRstPinMsg.setLabel(rst_pin_config_msg)


    global rfHostBufferConfigmenu
    rfHostBufferConfigmenu  = rfHostLib.createMenuSymbol("BMM_BUFFER", None)
    rfHostBufferConfigmenu.setLabel("Buffer Configuration")
    rfHostBufferConfigmenu.setDescription("PHY Buffer configuration")

    global phyIntegerBmmLargeBuffers
    phyIntegerBmmLargeBuffers = rfHostLib.createIntegerSymbol("PHY_INTEGER_BMMLARGEBUFFERS", rfHostBufferConfigmenu)
    phyIntegerBmmLargeBuffers.setLabel("Large Buffers")
    phyIntegerBmmLargeBuffers.setMin(3)
    phyIntegerBmmLargeBuffers.setMax(50)
    phyIntegerBmmLargeBuffers.setDefaultValue(3)

    global phyIntegerBmmSmallBuffers
    phyIntegerBmmSmallBuffers = rfHostLib.createIntegerSymbol("PHY_INTEGER_BMMSMALLBUFFERS", rfHostBufferConfigmenu)
    phyIntegerBmmSmallBuffers.setLabel("Small Buffers")
    phyIntegerBmmSmallBuffers.setMin(3)
    phyIntegerBmmSmallBuffers.setMax(50)
    phyIntegerBmmSmallBuffers.setDefaultValue(3)
    
    # === Compiler macros
    global preprocessorCompiler
    preprocessorCompiler = rfHostLib.createSettingSymbol("IEEE802154PHY_XC32_PREPRECESSOR", None)
    preprocessorCompiler.setValue("ENABLE_LARGE_BUFFER;ENABLE_QUEUE_CAPACITY;PHY_AT86RF233") #
    preprocessorCompiler.setCategory("C32")
    preprocessorCompiler.setKey("preprocessor-macros")
    preprocessorCompiler.setAppend(True, ";")
    preprocessorCompiler.setEnabled(True)

    #definitions 
    global rfSystemDefFile
    rfSystemDefFile = rfHostLib.createFileSymbol("RFPHY_DEFINITIONS", None)
    rfSystemDefFile.setType("STRING")
    rfSystemDefFile.setOutputName("core.LIST_SYSTEM_DEFINITIONS_H_EXTERNS")
    rfSystemDefFile.setSourcePath("driver/templates/common/RFPHY_definitions.h.ftl")
    rfSystemDefFile.setOverwrite(True)
    rfSystemDefFile.setMarkup(True)

    global idletaskh
    idletaskh = rfHostLib.createFileSymbol("APP_IDLE_TASK_HEADER", None)
    idletaskh.setSourcePath("/driver/templates/common/app_idle_task.h.ftl")
    idletaskh.setOutputName("app_idle_task.h")
    idletaskh.setDestPath('../../')
    idletaskh.setProjectPath('')
    idletaskh.setType("HEADER")
    idletaskh.setOverwrite(True)
    idletaskh.setMarkup(True)

    global idletaskc
    idletaskc = rfHostLib.createFileSymbol("APP_IDLE_TASK_SOURCE", None)
    idletaskc.setSourcePath("/driver/templates/common/app_idle_task.c.ftl")
    idletaskc.setOutputName("app_idle_task.c")
    idletaskc.setDestPath('../../')
    idletaskc.setProjectPath('')
    idletaskc.setType("SOURCE")
    idletaskc.setOverwrite(True)
    idletaskc.setMarkup(True)

    #initialization
    global phyInitFile
    phyInitFile = rfHostLib.createFileSymbol("PHY_INIT", None)
    phyInitFile.setType("STRING")
    phyInitFile.setOutputName("core.LIST_SYSTEM_INIT_INTERRUPTS")
    phyInitFile.setSourcePath("driver/templates/common/phy_initialize.c.ftl")
    phyInitFile.setMarkup(True)

    global semHandleInitFile
    semHandleInitFile = rfHostLib.createFileSymbol('OSAM_SEM_HANDLE_INIT', None)
    semHandleInitFile.setType('STRING')
    semHandleInitFile.setOutputName('core.LIST_SYSTEM_INIT_C_LIBRARY_INITIALIZATION_DATA')
    semHandleInitFile.setSourcePath('driver/templates/common/sem_handle_init.c.ftl')
    semHandleInitFile.setMarkup(True)

    global semCreateInitFile
    semCreateInitFile = rfHostLib.createFileSymbol("OSAL_SEM_CREATE_INIT", None)
    semCreateInitFile.setType("STRING")
    semCreateInitFile.setOutputName("core.LIST_SYSTEM_INIT_C_INITIALIZE_SYSTEM_SERVICES")
    semCreateInitFile.setSourcePath("driver/templates/common/sem_createInit.c.ftl")
    semCreateInitFile.setMarkup(True)

    global phyTasksDefC
    phyTasksDefC = rfHostLib.createFileSymbol('PHY_TASK_INITIALIZATION_C', None)
    phyTasksDefC.setType('STRING')
    phyTasksDefC.setOutputName('core.LIST_SYSTEM_RTOS_TASKS_C_DEFINITIONS')
    phyTasksDefC.setSourcePath('driver/templates/common/phy_task_def.c.ftl')
    phyTasksDefC.setMarkup(True)

    global phyTasksC
    phyTasksC = rfHostLib.createFileSymbol('PHY_TASKS_C', None)
    phyTasksC.setType('STRING')
    phyTasksC.setOutputName('core.LIST_SYSTEM_TASKS_C_CALL_LIB_TASKS')
    phyTasksC.setSourcePath('driver/templates/common/phy_task.c.ftl')
    phyTasksC.setMarkup(True)

    #app.h
    global rfAppH
    rfAppH = rfHostLib.createFileSymbol(None, None)
    rfAppH.setSourcePath('driver/templates/common/app.h.ftl')
    rfAppH.setOutputName('app.h')
    rfAppH.setOverwrite(True)
    rfAppH.setDestPath('../../')
    rfAppH.setProjectPath('')
    rfAppH.setType('HEADER')
    rfAppH.setMarkup(True)
    rfAppH.setEnabled(True)

    # #app.c
    # global rfAppC
    # rfAppC = rfHostLib.createFileSymbol(None, None)
    # rfAppC.setSourcePath('driver/templates/common/app.c.ftl')
    # rfAppC.setOutputName('app.c')
    # rfAppC.setOverwrite(True)
    # rfAppC.setDestPath('../../')
    # rfAppC.setProjectPath('')
    # rfAppC.setType('SOURCE')
    # rfAppC.setMarkup(True)
    # rfAppC.setEnabled(True)

    phyConfHeader = rfHostLib.createFileSymbol("PHY_CONF_HEADER", None)
    phyConfHeader.setSourcePath("/driver/templates/common/stack_config.h.ftl")
    phyConfHeader.setOutputName("stack_config.h")
    phyConfHeader.setDestPath('../../')
    phyConfHeader.setProjectPath('')
    phyConfHeader.setType("HEADER")
    phyConfHeader.setOverwrite(True)
    phyConfHeader.setMarkup(True)
    phyConfHeader.setEnabled(True)
    
    phyConfHeader = rfHostLib.createFileSymbol("APP_CONF_HEADER", None)
    phyConfHeader.setSourcePath("/driver/templates/common/app_config.h.ftl")
    phyConfHeader.setOutputName("app_config.h")
    phyConfHeader.setDestPath('../../')
    phyConfHeader.setProjectPath('')
    phyConfHeader.setType("HEADER")
    phyConfHeader.setOverwrite(True)
    phyConfHeader.setMarkup(True)
    phyConfHeader.setEnabled(True)

    #Add Framework_defs.h
    HeaderFile = rfHostLib.createFileSymbol(None, None)
    HeaderFile.setSourcePath('driver/software/app_fw//framework_defs.h')
    HeaderFile.setOutputName('framework_defs.h')
    HeaderFile.setOverwrite(True)
    HeaderFile.setDestPath('')
    HeaderFile.setProjectPath('config/' + configName)
    HeaderFile.setType('HEADER')
    HeaderFile.setEnabled(True)

    # Add osal_freertos_extend.h
    global freeRtosExtendH
    freeRtosExtendH = rfHostLib.createFileSymbol(None, None)
    freeRtosExtendH.setSourcePath('driver/software/app_fw/osal/osal_freertos_extend.h')
    freeRtosExtendH.setOutputName('osal_freertos_extend.h')
    freeRtosExtendH.setOverwrite(True)
    freeRtosExtendH.setDestPath('/osal')
    freeRtosExtendH.setProjectPath('config/' + configName + '/osal/')
    freeRtosExtendH.setType('HEADER')
    freeRtosExtendH.setEnabled(True)

    # Add osal_freertos_extend.c
    global freeRtosExtendC
    freeRtosExtendC = rfHostLib.createFileSymbol(None, None)
    freeRtosExtendC.setSourcePath("driver/software/app_fw/osal/osal_freertos_extend.c")
    freeRtosExtendC.setOutputName('osal_freertos_extend.c')
    freeRtosExtendC.setOverwrite(True)
    freeRtosExtendC.setDestPath('/osal')
    freeRtosExtendC.setProjectPath('config/' + configName + '/osal/')
    freeRtosExtendC.setType('SOURCE')
    freeRtosExtendC.setEnabled(True)


    #Add pal related files
    global palH
    palH = rfHostLib.createFileSymbol("pal_H", None)
    palH.setSourcePath("driver/templates/common/pal.h.ftl")
    palH.setOutputName("pal.h")
    palH.setDestPath("driver/IEEE_802154_PHY/pal/inc/")
    palH.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/pal/inc/")
    palH.setType("HEADER")
    palH.setOverwrite(True)
    palH.setMarkup(True)

    global palC
    palC = rfHostLib.createFileSymbol("pal_C", None)
    palC.setSourcePath("driver/software/pal/src/pal.c")
    palC.setOutputName("pal.c")
    palC.setDestPath("driver/IEEE_802154_PHY/pal/src/")
    palC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/pal/src/")
    palC.setType("SOURCE")
    palC.setOverwrite(True)

    global pal_inc
    pal_inc = rfHostLib.createSettingSymbol("IEEE802154PHY_INC_PATH_PAL", None)
    pal_inc.setValue("../src/config/" + configName + "/driver/IEEE_802154_PHY/pal/inc/;")
    pal_inc.setCategory("C32")
    pal_inc.setKey("extra-include-directories")
    pal_inc.setAppend(True, ";")
    pal_inc.setEnabled(True)

    global pal_inc_cpp
    pal_inc_cpp = rfHostLib.createSettingSymbol("IEEE802154PHY_INC_PATH_PAL_CPP", None)
    pal_inc_cpp.setValue("../src/config/" + configName + "/driver/IEEE_802154_PHY/pal/inc/;")
    pal_inc_cpp.setCategory("C32CPP")
    pal_inc_cpp.setKey("extra-include-directories")
    pal_inc_cpp.setAppend(True, ";")
    pal_inc_cpp.setEnabled(True)


    #Add resource related files
    global bmmH
    bmmH = rfHostLib.createFileSymbol("bmm_H", None) 
    bmmH.setSourcePath("driver/software/resources/buffer/inc/bmm.h")
    bmmH.setOutputName("bmm.h")
    bmmH.setDestPath("driver/IEEE_802154_PHY/resources/buffer/inc/")
    bmmH.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/resources/buffer/inc/")
    bmmH.setType("HEADER")
    bmmH.setOverwrite(True)

    global bmmC
    bmmC = rfHostLib.createFileSymbol("bmm_C", None)
    bmmC.setSourcePath("driver/software/resources/buffer/src/bmm.c")
    bmmC.setOutputName("bmm.c")
    bmmC.setDestPath("driver/IEEE_802154_PHY/resources/buffer/src/")
    bmmC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/resources/buffer/src/")
    bmmC.setType("SOURCE")
    bmmC.setOverwrite(True)

    global bmm_inc
    bmm_inc = rfHostLib.createSettingSymbol("IEEE802154PHY_INC_PATH_BMM", None)
    bmm_inc.setValue("../src/config/" + configName + "/driver/IEEE_802154_PHY/resources/buffer/inc/;")
    bmm_inc.setCategory("C32")
    bmm_inc.setKey("extra-include-directories")
    bmm_inc.setAppend(True, ";")
    bmm_inc.setEnabled(True)

    global bmm_inc_cpp
    bmm_inc_cpp = rfHostLib.createSettingSymbol("IEEE802154PHY_INC_PATH_BMM_CPP", None)
    bmm_inc_cpp.setValue("../src/config/" + configName + "/driver/IEEE_802154_PHY/resources/buffer/inc/;")
    bmm_inc_cpp.setCategory("C32CPP")
    bmm_inc_cpp.setKey("extra-include-directories")
    bmm_inc_cpp.setAppend(True, ";")
    bmm_inc_cpp.setEnabled(True)


    global qmmH
    qmmH = rfHostLib.createFileSymbol("qmm_H", None) 
    qmmH.setSourcePath("driver/software/resources/queue/inc/qmm.h")
    qmmH.setOutputName("qmm.h")
    qmmH.setDestPath("driver/IEEE_802154_PHY/resources/queue/inc/")
    qmmH.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/resources/queue/inc/")
    qmmH.setType("HEADER")
    qmmH.setOverwrite(True)

    global qmmC
    qmmC = rfHostLib.createFileSymbol("qmm_C", None)
    qmmC.setSourcePath("driver/software/resources/queue/src/qmm.c")
    qmmC.setOutputName("qmm.c")
    qmmC.setDestPath("driver/IEEE_802154_PHY/resources/queue/src/")
    qmmC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/resources/queue/src/")
    qmmC.setType("SOURCE")
    qmmC.setOverwrite(True)

    global qmm_inc
    qmm_inc = rfHostLib.createSettingSymbol("IEEE802154PHY_INC_PATH_QMM", None)
    qmm_inc.setValue("../src/config/" + configName + "/driver/IEEE_802154_PHY/resources/queue/inc/;")
    qmm_inc.setCategory("C32")
    qmm_inc.setKey("extra-include-directories")
    qmm_inc.setAppend(True, ";")
    qmm_inc.setEnabled(True)

    global qmm_inc_cpp
    qmm_inc_cpp = rfHostLib.createSettingSymbol("IEEE802154PHY_INC_PATH_QMM_CPP", None)
    qmm_inc_cpp.setValue("../src/config/" + configName + "/driver/IEEE_802154_PHY/resources/queue/inc/;")
    qmm_inc_cpp.setCategory("C32CPP")
    qmm_inc_cpp.setKey("extra-include-directories")
    qmm_inc_cpp.setAppend(True, ";")
    qmm_inc_cpp.setEnabled(True)

    #Add RF233 phy related files

    global phy_inc
    phy_inc = rfHostLib.createSettingSymbol("IEEE802154PHY_INC_PATH_PHY", None)
    phy_inc.setValue("../src/config/" + configName + "/driver/IEEE_802154_PHY/phy/inc/;")
    phy_inc.setCategory("C32")
    phy_inc.setKey("extra-include-directories")
    phy_inc.setAppend(True, ";")
    phy_inc.setEnabled(True)

    global phy_inc_cpp
    phy_inc_cpp = rfHostLib.createSettingSymbol("IEEE802154PHY_INC_PATH_PHY_CPP", None)
    phy_inc_cpp.setValue("../src/config/" + configName + "/driver/IEEE_802154_PHY/phy/inc/;")
    phy_inc_cpp.setCategory("C32CPP")
    phy_inc_cpp.setKey("extra-include-directories")
    phy_inc_cpp.setAppend(True, ";")
    phy_inc_cpp.setEnabled(True)

    global RF233_phyH
    RF233_phyH = rfHostLib.createFileSymbol("RF233_phy_H", None) 
    RF233_phyH.setSourcePath("driver/software/RF233/phy/inc/phy.h")
    RF233_phyH.setOutputName("phy.h")
    RF233_phyH.setDestPath("driver/IEEE_802154_PHY/phy/inc/")
    RF233_phyH.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/inc/")
    RF233_phyH.setType("HEADER")
    RF233_phyH.setOverwrite(True)   

    global RF233_ieee_phy_constH
    RF233_ieee_phy_constH = rfHostLib.createFileSymbol("RF233_ieee_phy_const_H", None) 
    RF233_ieee_phy_constH.setSourcePath("driver/software/RF233/phy/inc/ieee_phy_const.h")
    RF233_ieee_phy_constH.setOutputName("ieee_phy_const.h")
    RF233_ieee_phy_constH.setDestPath("driver/IEEE_802154_PHY/phy/inc/")
    RF233_ieee_phy_constH.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/inc/")
    RF233_ieee_phy_constH.setType("HEADER")
    RF233_ieee_phy_constH.setOverwrite(True)   

    global RF233_phy_configH
    RF233_phy_configH = rfHostLib.createFileSymbol("RF233_phy_config_H", None) 
    RF233_phy_configH.setSourcePath("driver/software/RF233/phy/inc/phy_config.h")
    RF233_phy_configH.setOutputName("phy_config.h")
    RF233_phy_configH.setDestPath("driver/IEEE_802154_PHY/phy/inc/")
    RF233_phy_configH.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/inc/")
    RF233_phy_configH.setType("HEADER")
    RF233_phy_configH.setOverwrite(True)   

    global RF233_phy_constantsH
    RF233_phy_constantsH = rfHostLib.createFileSymbol("RF233_phy_constants_H", None) 
    RF233_phy_constantsH.setSourcePath("driver/software/RF233/phy/inc/phy_constants.h")
    RF233_phy_constantsH.setOutputName("phy_constants.h")
    RF233_phy_constantsH.setDestPath("driver/IEEE_802154_PHY/phy/inc/")
    RF233_phy_constantsH.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/inc/")
    RF233_phy_constantsH.setType("HEADER")
    RF233_phy_constantsH.setOverwrite(True) 

    global RF233_phy_tasksH
    RF233_phy_tasksH = rfHostLib.createFileSymbol("RF233_phy_tasks_H", None) 
    RF233_phy_tasksH.setSourcePath("driver/software/RF233/phy/inc/phy_tasks.h")
    RF233_phy_tasksH.setOutputName("phy_tasks.h")
    RF233_phy_tasksH.setDestPath("driver/IEEE_802154_PHY/phy/inc/")
    RF233_phy_tasksH.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/inc/")
    RF233_phy_tasksH.setType("HEADER")
    RF233_phy_tasksH.setOverwrite(True)   

    global RF233_phy_internalH
    RF233_phy_internalH = rfHostLib.createFileSymbol("RF233_phy_internal_H", None) 
    RF233_phy_internalH.setSourcePath("driver/software/RF233/phy/at86rf233/inc/phy_internal.h")
    RF233_phy_internalH.setOutputName("phy_internal.h")
    RF233_phy_internalH.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/inc/")
    RF233_phy_internalH.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/inc/")
    RF233_phy_internalH.setType("HEADER")
    RF233_phy_internalH.setOverwrite(True)  

    global RF233_phy_irq_handlerH
    RF233_phy_irq_handlerH = rfHostLib.createFileSymbol("RF233_phy_irq_handler_H", None) 
    RF233_phy_irq_handlerH.setSourcePath("driver/software/RF233/phy/at86rf233/inc/phy_irq_handler.h")
    RF233_phy_irq_handlerH.setOutputName("phy_irq_handler.h")
    RF233_phy_irq_handlerH.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/inc/")
    RF233_phy_irq_handlerH.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/inc/")
    RF233_phy_irq_handlerH.setType("HEADER")
    RF233_phy_irq_handlerH.setOverwrite(True)  

    global RF233_phy_pibH
    RF233_phy_pibH = rfHostLib.createFileSymbol("RF233_phy_pib_H", None) 
    RF233_phy_pibH.setSourcePath("driver/software/RF233/phy/at86rf233/inc/phy_pib.h")
    RF233_phy_pibH.setOutputName("phy_pib.h")
    RF233_phy_pibH.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/inc/")
    RF233_phy_pibH.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/inc/")
    RF233_phy_pibH.setType("HEADER")
    RF233_phy_pibH.setOverwrite(True)  

    global RF233_phy_rxH
    RF233_phy_rxH = rfHostLib.createFileSymbol("RF233_phy_rx_H", None) 
    RF233_phy_rxH.setSourcePath("driver/software/RF233/phy/at86rf233/inc/phy_rx.h")
    RF233_phy_rxH.setOutputName("phy_rx.h")
    RF233_phy_rxH.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/inc/")
    RF233_phy_rxH.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/inc/")
    RF233_phy_rxH.setType("HEADER")
    RF233_phy_rxH.setOverwrite(True)  


    global RF233_phy_trx_reg_accessH
    RF233_phy_trx_reg_accessH = rfHostLib.createFileSymbol("RF233_phy_trx_reg_access_H", None) 
    RF233_phy_trx_reg_accessH.setSourcePath("driver/templates/RF233/phy_trx_reg_access.h.ftl")
    RF233_phy_trx_reg_accessH.setOutputName("phy_trx_reg_access.h")
    RF233_phy_trx_reg_accessH.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/inc/")
    RF233_phy_trx_reg_accessH.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/inc/")
    RF233_phy_trx_reg_accessH.setType("HEADER") 
    RF233_phy_trx_reg_accessH.setMarkup(True) 



    global RF233_phy_txH
    RF233_phy_txH = rfHostLib.createFileSymbol("RF233_phy_tx_H", None) 
    RF233_phy_txH.setSourcePath("driver/software/RF233/phy/at86rf233/inc/phy_tx.h")
    RF233_phy_txH.setOutputName("phy_tx.h")
    RF233_phy_txH.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/inc/")
    RF233_phy_txH.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/inc/")
    RF233_phy_txH.setType("HEADER")
    RF233_phy_txH.setOverwrite(True)

    global at86rf233H
    at86rf233H = rfHostLib.createFileSymbol("at86rf233_H", None) 
    at86rf233H.setSourcePath("driver/software/RF233/phy/at86rf233/inc/AT86RF233.h")
    at86rf233H.setOutputName("at86rf.h")
    at86rf233H.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/inc/")
    at86rf233H.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/inc/")
    at86rf233H.setType("HEADER")
    at86rf233H.setOverwrite(True)


    global RF233_phyC
    RF233_phyC = rfHostLib.createFileSymbol("RF233_phy_C", None)
    RF233_phyC.setSourcePath("driver/templates/RF233/phy.c.ftl")
    RF233_phyC.setOutputName("phy.c")
    RF233_phyC.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF233_phyC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF233_phyC.setType("SOURCE")
    RF233_phyC.setOverwrite(True)
    RF233_phyC.setMarkup(True)

    global RF233_phy_edC
    RF233_phy_edC = rfHostLib.createFileSymbol("RF233_phy_ed_C", None)
    RF233_phy_edC.setSourcePath("driver/software/RF233/phy/at86rf233/src/phy_ed.c")
    RF233_phy_edC.setOutputName("phy_ed.c")
    RF233_phy_edC.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF233_phy_edC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF233_phy_edC.setType("SOURCE")
    RF233_phy_edC.setOverwrite(True)

    global RF233_phy_helperC
    RF233_phy_helperC = rfHostLib.createFileSymbol("RF233_phy_helper_C", None)
    RF233_phy_helperC.setSourcePath("driver/software/RF233/phy/at86rf233/src/phy_helper.c")
    RF233_phy_helperC.setOutputName("phy_helper.c")
    RF233_phy_helperC.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF233_phy_helperC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF233_phy_helperC.setType("SOURCE")
    RF233_phy_helperC.setOverwrite(True)

    global RF233_phy_initC
    RF233_phy_initC = rfHostLib.createFileSymbol("RF233_phy_init_C", None)
    RF233_phy_initC.setSourcePath("driver/templates/RF233/phy_init.c.ftl")
    RF233_phy_initC.setOutputName("phy_init.c")
    RF233_phy_initC.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF233_phy_initC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF233_phy_initC.setType("SOURCE")
    RF233_phy_initC.setOverwrite(True)
    RF233_phy_initC.setMarkup(True)

    global RF233_phy_irq_handlerC
    RF233_phy_irq_handlerC = rfHostLib.createFileSymbol("RF233_phy_irq_handler_C", None)
    RF233_phy_irq_handlerC.setSourcePath("driver/software/RF233/phy/at86rf233/src/phy_irq_handler.c")
    RF233_phy_irq_handlerC.setOutputName("phy_irq_handler.c")
    RF233_phy_irq_handlerC.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF233_phy_irq_handlerC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF233_phy_irq_handlerC.setType("SOURCE")
    RF233_phy_irq_handlerC.setOverwrite(True)

    global RF233_phy_trx_reg_accessC
    RF233_phy_trx_reg_accessC = rfHostLib.createFileSymbol("RF233_phy_trx_reg_access_C", None) 
    RF233_phy_trx_reg_accessC.setSourcePath("driver/templates/RF233/phy_trx_reg_access.c.ftl")
    RF233_phy_trx_reg_accessC.setOutputName("phy_trx_reg_access.c")
    RF233_phy_trx_reg_accessC.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF233_phy_trx_reg_accessC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF233_phy_trx_reg_accessC.setType("SOURCE")
    RF233_phy_trx_reg_accessC.setOverwrite(True)  
    RF233_phy_trx_reg_accessC.setMarkup(True)

    global RF215_phy_trx_reg_accessC
    RF215_phy_trx_reg_accessC = rfHostLib.createFileSymbol("RF215_phy_trx_reg_accessC", None) 
    RF215_phy_trx_reg_accessC.setSourcePath("driver/templates/RF215/phy_trx_reg_access.c.ftl")
    RF215_phy_trx_reg_accessC.setOutputName("trx_access_2.c")
    RF215_phy_trx_reg_accessC.setDestPath("driver/IEEE_802154_PHY/phy/at86rf215/src/")
    RF215_phy_trx_reg_accessC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf215/src/")
    RF215_phy_trx_reg_accessC.setType("SOURCE")
    RF215_phy_trx_reg_accessC.setOverwrite(True)  
    RF215_phy_trx_reg_accessC.setMarkup(True)
    RF215_phy_trx_reg_accessC.setEnabled(False)
    file_list_RF215.append(RF215_phy_trx_reg_accessC)

    global RF233_phy_pibC
    RF233_phy_pibC = rfHostLib.createFileSymbol("RF233_phy_pib_C", None)
    RF233_phy_pibC.setSourcePath("driver/software/RF233/phy/at86rf233/src/phy_pib.c")
    RF233_phy_pibC.setOutputName("phy_pib.c")
    RF233_phy_pibC.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF233_phy_pibC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF233_phy_pibC.setType("SOURCE")
    RF233_phy_pibC.setOverwrite(True)

    global RF233_phy_pwr_mgmtC
    RF233_phy_pwr_mgmtC = rfHostLib.createFileSymbol("RF233_phy_pwr_mgmt_C", None)
    RF233_phy_pwr_mgmtC.setSourcePath("driver/software/RF233/phy/at86rf233/src/phy_pwr_mgmt.c")
    RF233_phy_pwr_mgmtC.setOutputName("phy_pwr_mgmt.c")
    RF233_phy_pwr_mgmtC.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF233_phy_pwr_mgmtC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF233_phy_pwr_mgmtC.setType("SOURCE")
    RF233_phy_pwr_mgmtC.setOverwrite(True)

    global RF233_phy_rxC
    RF233_phy_rxC = rfHostLib.createFileSymbol("RF233_phy_rx_C", None)
    RF233_phy_rxC.setSourcePath("driver/software/RF233/phy/at86rf233/src/phy_rx.c")
    RF233_phy_rxC.setOutputName("phy_rx.c")
    RF233_phy_rxC.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF233_phy_rxC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF233_phy_rxC.setType("SOURCE")
    RF233_phy_rxC.setOverwrite(True)

    global RF233_phy_rx_enableC
    RF233_phy_rx_enableC = rfHostLib.createFileSymbol("RF233_phy_rx_enable_C", None)
    RF233_phy_rx_enableC.setSourcePath("driver/software/RF233/phy/at86rf233/src/phy_rx_enable.c")
    RF233_phy_rx_enableC.setOutputName("phy_rx_enable.c")
    RF233_phy_rx_enableC.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF233_phy_rx_enableC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF233_phy_rx_enableC.setType("SOURCE")
    RF233_phy_rx_enableC.setOverwrite(True)


    global RF233_phy_txC
    RF233_phy_txC = rfHostLib.createFileSymbol("RF233_phy_tx_C", None)
    RF233_phy_txC.setSourcePath("driver/software/RF233/phy/at86rf233/src/phy_tx.c")
    RF233_phy_txC.setOutputName("phy_tx.c")
    RF233_phy_txC.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF233_phy_txC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF233_phy_txC.setType("SOURCE")
    RF233_phy_txC.setOverwrite(True)

    global RF233_tfaC
    RF233_tfaC = rfHostLib.createFileSymbol("RF233_tfa_C", None)
    RF233_tfaC.setSourcePath("driver/software/RF233/phy/at86rf233/src/tfa.c")
    RF233_tfaC.setOutputName("tfa.c")
    RF233_tfaC.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF233_tfaC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF233_tfaC.setType("SOURCE")
    RF233_tfaC.setOverwrite(True)

    global RF233_phy_ed_end_cbC
    RF233_phy_ed_end_cbC = rfHostLib.createFileSymbol("RF233_phy_ed_end_cb_C", None)
    RF233_phy_ed_end_cbC.setSourcePath("driver/software/RF233/phy/src/phy_ed_end_cb.c")
    RF233_phy_ed_end_cbC.setOutputName("phy_ed_end_cb.c")
    RF233_phy_ed_end_cbC.setDestPath("driver/IEEE_802154_PHY/phy/src/")
    RF233_phy_ed_end_cbC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/src/")
    RF233_phy_ed_end_cbC.setType("SOURCE")
    RF233_phy_ed_end_cbC.setOverwrite(True)

    global RF233_phy_rx_frame_cbC
    RF233_phy_rx_frame_cbC = rfHostLib.createFileSymbol("RF233_phy_rx_frame_cb_C", None)
    RF233_phy_rx_frame_cbC.setSourcePath("driver/software/RF233/phy/src/phy_rx_frame_cb.c")
    RF233_phy_rx_frame_cbC.setOutputName("phy_rx_frame_cb.c")
    RF233_phy_rx_frame_cbC.setDestPath("driver/IEEE_802154_PHY/phy/src/")
    RF233_phy_rx_frame_cbC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/src/")
    RF233_phy_rx_frame_cbC.setType("SOURCE")
    RF233_phy_rx_frame_cbC.setOverwrite(True)

    global RF233_phy_taskC
    RF233_phy_taskC = rfHostLib.createFileSymbol("RF233_phy_task_C", None)
    RF233_phy_taskC.setSourcePath("driver/software/RF233/phy/src/phy_task.c")
    RF233_phy_taskC.setOutputName("phy_task.c")
    RF233_phy_taskC.setDestPath("driver/IEEE_802154_PHY/phy/src/")
    RF233_phy_taskC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/src/")
    RF233_phy_taskC.setType("SOURCE")
    RF233_phy_taskC.setOverwrite(True)

    global RF233_phy_tx_frame_done_cbC
    RF233_phy_tx_frame_done_cbC = rfHostLib.createFileSymbol("RF233_phy_tx_frame_done_cb_C", None)
    RF233_phy_tx_frame_done_cbC.setSourcePath("driver/software/RF233/phy/src/phy_tx_frame_done_cb.c")
    RF233_phy_tx_frame_done_cbC.setOutputName("phy_tx_frame_done_cb.c")
    RF233_phy_tx_frame_done_cbC.setDestPath("driver/IEEE_802154_PHY/phy/src/")
    RF233_phy_tx_frame_done_cbC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/src/")
    RF233_phy_tx_frame_done_cbC.setType("SOURCE")
    RF233_phy_tx_frame_done_cbC.setOverwrite(True)
    print("RF233_phy_tx_frame_done_cbC",RF233_phy_tx_frame_done_cbC)
 

    #Add RF212b phy related files
    global RF212b_phyH
    RF212b_phyH = rfHostLib.createFileSymbol("RF212b_phy_H", None) 
    RF212b_phyH.setSourcePath("driver/software/RF212b/phy/inc/phy.h")
    RF212b_phyH.setOutputName("phy.h")
    RF212b_phyH.setDestPath("driver/IEEE_802154_PHY/phy/inc/")
    RF212b_phyH.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/inc/")
    RF212b_phyH.setType("HEADER")
    RF212b_phyH.setOverwrite(True)   
    RF212b_phyH.setEnabled(False)

    global RF212b_ieee_phy_constH
    RF212b_ieee_phy_constH = rfHostLib.createFileSymbol("RF212b_ieee_phy_const_H", None) 
    RF212b_ieee_phy_constH.setSourcePath("driver/software/RF212b/phy/inc/ieee_phy_const.h")
    RF212b_ieee_phy_constH.setOutputName("ieee_phy_const.h")
    RF212b_ieee_phy_constH.setDestPath("driver/IEEE_802154_PHY/phy/inc/")
    RF212b_ieee_phy_constH.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/inc/")
    RF212b_ieee_phy_constH.setType("HEADER")
    RF212b_ieee_phy_constH.setOverwrite(True)
    RF212b_ieee_phy_constH.setEnabled(False)

    global RF212b_phy_configH
    RF212b_phy_configH = rfHostLib.createFileSymbol("RF212b_phy_config_H", None) 
    RF212b_phy_configH.setSourcePath("driver/software/RF212b/phy/inc/phy_config.h")
    RF212b_phy_configH.setOutputName("phy_config.h")
    RF212b_phy_configH.setDestPath("driver/IEEE_802154_PHY/phy/inc/")
    RF212b_phy_configH.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/inc/")
    RF212b_phy_configH.setType("HEADER")
    RF212b_phy_configH.setOverwrite(True) 
    RF212b_phy_configH.setEnabled(False)  

    global RF212b_phy_constantsH
    RF212b_phy_constantsH = rfHostLib.createFileSymbol("RF212b_phy_constants_H", None) 
    RF212b_phy_constantsH.setSourcePath("driver/software/RF212b/phy/inc/phy_constants.h")
    RF212b_phy_constantsH.setOutputName("phy_constants.h")
    RF212b_phy_constantsH.setDestPath("driver/IEEE_802154_PHY/phy/inc/")
    RF212b_phy_constantsH.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/inc/")
    RF212b_phy_constantsH.setType("HEADER")
    RF212b_phy_constantsH.setOverwrite(True) 
    RF212b_phy_constantsH.setEnabled(False)

    global RF212b_phy_tasksH
    RF212b_phy_tasksH = rfHostLib.createFileSymbol("RF212b_phy_tasks_H", None) 
    RF212b_phy_tasksH.setSourcePath("driver/software/RF212b/phy/inc/phy_tasks.h")
    RF212b_phy_tasksH.setOutputName("phy_tasks.h")
    RF212b_phy_tasksH.setDestPath("driver/IEEE_802154_PHY/phy/inc/")
    RF212b_phy_tasksH.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/inc/")
    RF212b_phy_tasksH.setType("HEADER")
    RF212b_phy_tasksH.setOverwrite(True)  
    RF212b_phy_tasksH.setEnabled(False) 

    global RF212b_phy_internalH
    RF212b_phy_internalH = rfHostLib.createFileSymbol("RF212b_phy_internal_H", None) 
    RF212b_phy_internalH.setSourcePath("driver/software/RF212b/phy/at86rf212b/inc/phy_internal.h")
    RF212b_phy_internalH.setOutputName("phy_internal.h")
    RF212b_phy_internalH.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/inc/")
    RF212b_phy_internalH.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/inc/")
    RF212b_phy_internalH.setType("HEADER")
    RF212b_phy_internalH.setOverwrite(True)
    RF212b_phy_internalH.setEnabled(False) 

    global RF212b_phy_irq_handlerH
    RF212b_phy_irq_handlerH = rfHostLib.createFileSymbol("RF212b_phy_irq_handler_H", None) 
    RF212b_phy_irq_handlerH.setSourcePath("driver/software/RF212b/phy/at86rf212b/inc/phy_irq_handler.h")
    RF212b_phy_irq_handlerH.setOutputName("phy_irq_handler.h")
    RF212b_phy_irq_handlerH.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/inc/")
    RF212b_phy_irq_handlerH.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/inc/")
    RF212b_phy_irq_handlerH.setType("HEADER")
    RF212b_phy_irq_handlerH.setOverwrite(True)
    RF212b_phy_irq_handlerH.setEnabled(False)  

    global RF212b_phy_pibH
    RF212b_phy_pibH = rfHostLib.createFileSymbol("RF212b_phy_pib_H", None) 
    RF212b_phy_pibH.setSourcePath("driver/software/RF212b/phy/at86rf212b/inc/phy_pib.h")
    RF212b_phy_pibH.setOutputName("phy_pib.h")
    RF212b_phy_pibH.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/inc/")
    RF212b_phy_pibH.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/inc/")
    RF212b_phy_pibH.setType("HEADER")
    RF212b_phy_pibH.setOverwrite(True)  
    RF212b_phy_pibH.setEnabled(False)

    global RF212b_phy_rxH
    RF212b_phy_rxH = rfHostLib.createFileSymbol("RF212b_phy_rx_H", None) 
    RF212b_phy_rxH.setSourcePath("driver/software/RF212b/phy/at86rf212b/inc/phy_rx.h")
    RF212b_phy_rxH.setOutputName("phy_rx.h")
    RF212b_phy_rxH.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/inc/")
    RF212b_phy_rxH.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/inc/")
    RF212b_phy_rxH.setType("HEADER")
    RF212b_phy_rxH.setOverwrite(True)  
    RF212b_phy_rxH.setEnabled(False)


    global RF212b_phy_trx_reg_accessH
    RF212b_phy_trx_reg_accessH = rfHostLib.createFileSymbol("RF212b_phy_trx_reg_access_H", None) 
    RF212b_phy_trx_reg_accessH.setSourcePath("driver/templates/RF212b/phy_trx_reg_access.h.ftl")
    RF212b_phy_trx_reg_accessH.setOutputName("phy_trx_reg_access.h")
    RF212b_phy_trx_reg_accessH.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/inc/")
    RF212b_phy_trx_reg_accessH.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/inc/")
    RF212b_phy_trx_reg_accessH.setType("HEADER") 
    RF212b_phy_trx_reg_accessH.setMarkup(True) 
    RF212b_phy_trx_reg_accessH.setEnabled(False)



    global RF212b_phy_txH
    RF212b_phy_txH = rfHostLib.createFileSymbol("RF212b_phy_tx_H", None) 
    RF212b_phy_txH.setSourcePath("driver/software/RF212b/phy/at86rf212b/inc/phy_tx.h")
    RF212b_phy_txH.setOutputName("phy_tx.h")
    RF212b_phy_txH.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/inc/")
    RF212b_phy_txH.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/inc/")
    RF212b_phy_txH.setType("HEADER")
    RF212b_phy_txH.setOverwrite(True)
    RF212b_phy_txH.setEnabled(False)



    global at86rf212bH
    at86rf212bH = rfHostLib.createFileSymbol("RF212b_at86rf212b_H", None) 
    at86rf212bH.setSourcePath("driver/software/RF212b/phy/at86rf212b/inc/AT86RF212b.h")
    at86rf212bH.setOutputName("at86rf.h")
    at86rf212bH.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/inc/")
    at86rf212bH.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/inc/")
    at86rf212bH.setType("HEADER")
    at86rf212bH.setOverwrite(True)
    at86rf212bH.setEnabled(False)



    global RF212b_phyC
    RF212b_phyC = rfHostLib.createFileSymbol("RF212b_phy_C", None)
    RF212b_phyC.setSourcePath("driver/templates/RF212b/phy.c.ftl")
    RF212b_phyC.setOutputName("phy.c")
    RF212b_phyC.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF212b_phyC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF212b_phyC.setType("SOURCE")
    RF212b_phyC.setOverwrite(True)
    RF212b_phyC.setMarkup(True)
    RF212b_phyC.setEnabled(False)

    global RF212b_phy_edC
    RF212b_phy_edC = rfHostLib.createFileSymbol("RF212b_phy_ed_C", None)
    RF212b_phy_edC.setSourcePath("driver/software/RF212b/phy/at86rf212b/src/phy_ed.c")
    RF212b_phy_edC.setOutputName("phy_ed.c")
    RF212b_phy_edC.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF212b_phy_edC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF212b_phy_edC.setType("SOURCE")
    RF212b_phy_edC.setOverwrite(True)
    RF212b_phy_edC.setEnabled(False)

    global RF212b_phy_helperC
    RF212b_phy_helperC = rfHostLib.createFileSymbol("RF212b_phy_helper_C", None)
    RF212b_phy_helperC.setSourcePath("driver/software/RF212b/phy/at86rf212b/src/phy_helper.c")
    RF212b_phy_helperC.setOutputName("phy_helper.c")
    RF212b_phy_helperC.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF212b_phy_helperC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF212b_phy_helperC.setType("SOURCE")
    RF212b_phy_helperC.setOverwrite(True)
    RF212b_phy_helperC.setEnabled(False)

    global RF212b_phy_initC
    RF212b_phy_initC = rfHostLib.createFileSymbol("RF212b_phy_init_C", None)
    RF212b_phy_initC.setSourcePath("driver/templates/RF212b/phy_init.c.ftl")
    RF212b_phy_initC.setOutputName("phy_init.c")
    RF212b_phy_initC.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF212b_phy_initC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF212b_phy_initC.setType("SOURCE")
    RF212b_phy_initC.setOverwrite(True)
    RF212b_phy_initC.setMarkup(True)
    RF212b_phy_initC.setEnabled(False)

    global RF212b_phy_irq_handlerC
    RF212b_phy_irq_handlerC = rfHostLib.createFileSymbol("RF212b_phy_irq_handler_C", None)
    RF212b_phy_irq_handlerC.setSourcePath("driver/software/RF212b/phy/at86rf212b/src/phy_irq_handler.c")
    RF212b_phy_irq_handlerC.setOutputName("phy_irq_handler.c")
    RF212b_phy_irq_handlerC.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF212b_phy_irq_handlerC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF212b_phy_irq_handlerC.setType("SOURCE")
    RF212b_phy_irq_handlerC.setOverwrite(True)
    RF212b_phy_irq_handlerC.setEnabled(False)

    global RF212b_phy_trx_reg_accessC
    RF212b_phy_trx_reg_accessC = rfHostLib.createFileSymbol("RF212b_phy_trx_reg_access_C", None) 
    RF212b_phy_trx_reg_accessC.setSourcePath("driver/templates/RF212b/phy_trx_reg_access.c.ftl")
    RF212b_phy_trx_reg_accessC.setOutputName("phy_trx_reg_access.c")
    RF212b_phy_trx_reg_accessC.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF212b_phy_trx_reg_accessC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF212b_phy_trx_reg_accessC.setType("SOURCE")
    RF212b_phy_trx_reg_accessC.setOverwrite(True)  
    RF212b_phy_trx_reg_accessC.setMarkup(True)
    RF212b_phy_trx_reg_accessC.setEnabled(False)

    global RF212b_phy_pibC
    RF212b_phy_pibC = rfHostLib.createFileSymbol("RF212b_phy_pib_C", None)
    RF212b_phy_pibC.setSourcePath("driver/software/RF212b/phy/at86rf212b/src/phy_pib.c")
    RF212b_phy_pibC.setOutputName("phy_pib.c")
    RF212b_phy_pibC.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF212b_phy_pibC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF212b_phy_pibC.setType("SOURCE")
    RF212b_phy_pibC.setOverwrite(True)
    RF212b_phy_pibC.setEnabled(False)

    global RF212b_phy_pwr_mgmtC
    RF212b_phy_pwr_mgmtC = rfHostLib.createFileSymbol("RF212b_phy_pwr_mgmt_C", None)
    RF212b_phy_pwr_mgmtC.setSourcePath("driver/software/RF212b/phy/at86rf212b/src/phy_pwr_mgmt.c")
    RF212b_phy_pwr_mgmtC.setOutputName("phy_pwr_mgmt.c")
    RF212b_phy_pwr_mgmtC.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF212b_phy_pwr_mgmtC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF212b_phy_pwr_mgmtC.setType("SOURCE")
    RF212b_phy_pwr_mgmtC.setOverwrite(True)
    RF212b_phy_pwr_mgmtC.setEnabled(False)

    global RF212b_phy_rxC
    RF212b_phy_rxC = rfHostLib.createFileSymbol("RF212b_phy_rx_C", None)
    RF212b_phy_rxC.setSourcePath("driver/software/RF212b/phy/at86rf212b/src/phy_rx.c")
    RF212b_phy_rxC.setOutputName("phy_rx.c")
    RF212b_phy_rxC.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF212b_phy_rxC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF212b_phy_rxC.setType("SOURCE")
    RF212b_phy_rxC.setOverwrite(True)
    RF212b_phy_rxC.setEnabled(False)

    global RF212b_phy_rx_enableC
    RF212b_phy_rx_enableC = rfHostLib.createFileSymbol("RF212b_phy_rx_enable_C", None)
    RF212b_phy_rx_enableC.setSourcePath("driver/software/RF212b/phy/at86rf212b/src/phy_rx_enable.c")
    RF212b_phy_rx_enableC.setOutputName("phy_rx_enable.c")
    RF212b_phy_rx_enableC.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF212b_phy_rx_enableC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF212b_phy_rx_enableC.setType("SOURCE")
    RF212b_phy_rx_enableC.setOverwrite(True)
    RF212b_phy_rx_enableC.setEnabled(False)


    global RF212b_phy_txC
    RF212b_phy_txC = rfHostLib.createFileSymbol("RF212b_phy_tx_C", None)
    RF212b_phy_txC.setSourcePath("driver/software/RF212b/phy/at86rf212b/src/phy_tx.c")
    RF212b_phy_txC.setOutputName("phy_tx.c")
    RF212b_phy_txC.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF212b_phy_txC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF212b_phy_txC.setType("SOURCE")
    RF212b_phy_txC.setOverwrite(True)
    RF212b_phy_txC.setEnabled(False)

    global RF212b_tfaC
    RF212b_tfaC = rfHostLib.createFileSymbol("RF212b_tfa_C", None)
    RF212b_tfaC.setSourcePath("driver/software/RF212b/phy/at86rf212b/src/tfa.c")
    RF212b_tfaC.setOutputName("tfa.c")
    RF212b_tfaC.setDestPath("driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF212b_tfaC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/at86rf/src/")
    RF212b_tfaC.setType("SOURCE")
    RF212b_tfaC.setOverwrite(True)
    RF212b_tfaC.setEnabled(False)

    global RF212b_phy_ed_end_cbC
    RF212b_phy_ed_end_cbC = rfHostLib.createFileSymbol("RF212b_phy_ed_end_cb_C", None)
    RF212b_phy_ed_end_cbC.setSourcePath("driver/software/RF212b/phy/src/phy_ed_end_cb.c")
    RF212b_phy_ed_end_cbC.setOutputName("phy_ed_end_cb.c")
    RF212b_phy_ed_end_cbC.setDestPath("driver/IEEE_802154_PHY/phy/src/")
    RF212b_phy_ed_end_cbC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/src/")
    RF212b_phy_ed_end_cbC.setType("SOURCE")
    RF212b_phy_ed_end_cbC.setOverwrite(True)
    RF212b_phy_ed_end_cbC.setEnabled(False)

    global RF212b_phy_rx_frame_cbC
    RF212b_phy_rx_frame_cbC = rfHostLib.createFileSymbol("RF212b_phy_rx_frame_cb_C", None)
    RF212b_phy_rx_frame_cbC.setSourcePath("driver/software/RF212b/phy/src/phy_rx_frame_cb.c")
    RF212b_phy_rx_frame_cbC.setOutputName("phy_rx_frame_cb.c")
    RF212b_phy_rx_frame_cbC.setDestPath("driver/IEEE_802154_PHY/phy/src/")
    RF212b_phy_rx_frame_cbC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/src/")
    RF212b_phy_rx_frame_cbC.setType("SOURCE")
    RF212b_phy_rx_frame_cbC.setOverwrite(True)
    RF212b_phy_rx_frame_cbC.setEnabled(False)

    global RF212b_phy_taskC
    RF212b_phy_taskC = rfHostLib.createFileSymbol("RF212b_phy_task_C", None)
    RF212b_phy_taskC.setSourcePath("driver/software/RF212b/phy/src/phy_task.c")
    RF212b_phy_taskC.setOutputName("phy_task.c")
    RF212b_phy_taskC.setDestPath("driver/IEEE_802154_PHY/phy/src/")
    RF212b_phy_taskC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/src/")
    RF212b_phy_taskC.setType("SOURCE")
    RF212b_phy_taskC.setOverwrite(True)
    RF212b_phy_taskC.setEnabled(False)

    global RF212b_phy_tx_frame_done_cbC
    RF212b_phy_tx_frame_done_cbC = rfHostLib.createFileSymbol("RF212b_phy_tx_frame_done_cb_C", None)
    RF212b_phy_tx_frame_done_cbC.setSourcePath("driver/software/RF212b/phy/src/phy_tx_frame_done_cb.c")
    RF212b_phy_tx_frame_done_cbC.setOutputName("phy_tx_frame_done_cb.c")
    RF212b_phy_tx_frame_done_cbC.setDestPath("driver/IEEE_802154_PHY/phy/src/")
    RF212b_phy_tx_frame_done_cbC.setProjectPath('config/' + configName + "/driver/IEEE_802154_PHY/phy/src/")
    RF212b_phy_tx_frame_done_cbC.setType("SOURCE")
    RF212b_phy_tx_frame_done_cbC.setOverwrite(True)
    RF212b_phy_tx_frame_done_cbC.setEnabled(False)



    # === Treat warnings as errors
    mimacWarnAsErr = rfHostLib.createSettingSymbol("MIWI_GCC_WARN_ERROR", None)
    mimacWarnAsErr.setValue("false")
    mimacWarnAsErr.setCategory("C32")
    mimacWarnAsErr.setKey("make-warnings-into-errors")

    # === Set optimization level
    mimacOptLevel = rfHostLib.createSettingSymbol("PET_LEVEL", None)
    mimacOptLevel.setValue("-O1")
    mimacOptLevel.setCategory("C32")
    mimacOptLevel.setKey("optimization-level")

  
    conditionAlwaysInclude = [True, None, []]
##RF215 file addition
    rf215_phy_src_files = [
    ["phy/src/phy_ed_end_cb.c", condTrxTypeRF215],
    ["phy/src/phy_rx_frame_cb.c", condTrxTypeRF215],
    ["phy/src/phy_task.c", condTrxTypeRF215],
    ["phy/src/phy_tx_frame_done_cb.c", condTrxTypeRF215],    
    ]

    rf215_phy_inc_files = [
    ["phy/inc/ieee_phy_const.h", condTrxTypeRF215],
    ["phy/inc/phy.h", condTrxTypeRF215],
    ["phy/inc/phy_constants.h", condTrxTypeRF215],
    ["phy/inc/phy_tasks.h", condTrxTypeRF215],    
    ]

    rf215_phy_at86rf_src_files = [
    ["phy/at86rf215/src/tal_4g_utils.c", condTrxTypeRF215],
    ["phy/at86rf215/src/tal_auto_ack.c", condTrxTypeRF215],
    ["phy/at86rf215/src/tal_auto_csma.c", condTrxTypeRF215],
    ["phy/at86rf215/src/tal_auto_rx.c", condTrxTypeRF215],    
    ["phy/at86rf215/src/tal.c", condTrxTypeRF215],
    ["phy/at86rf215/src/tal_auto_tx.c", condTrxTypeRF215],
    ["phy/at86rf215/src/tal_ed.c", condTrxTypeRF215],
    ["phy/at86rf215/src/tal_fe.c", condTrxTypeRF215],  
    ["phy/at86rf215/src/tal_ftn.c", condTrxTypeRF215],  
    ["phy/at86rf215/src/tal_helper_2.c", condTrxTypeRF215],  
    ["phy/at86rf215/src/tal_init.c", condTrxTypeRF215],  
    ["phy/at86rf215/src/tal_irq_handler.c", condTrxTypeRF215],  
    ["phy/at86rf215/src/tal_mode_switch.c", condTrxTypeRF215],
    ["phy/at86rf215/src/tal_multitrx_interface.c", condTrxTypeRF215],
    ["phy/at86rf215/src/tal_phy_cfg.c", condTrxTypeRF215], 
    ["phy/at86rf215/src/tal_pib.c", condTrxTypeRF215],
    ["phy/at86rf215/src/tal_pwr_mgmt.c", condTrxTypeRF215], 
    ["phy/at86rf215/src/tal_rand.c", condTrxTypeRF215],  
    ["phy/at86rf215/src/tal_rx_enable.c", condTrxTypeRF215],
    ["phy/at86rf215/src/tfa.c", condTrxTypeRF215], 
    ["phy/at86rf215/src/tfa_batmon.c", condTrxTypeRF215], 
    # ["phy/at86rf215/src/trx_access_2.c", condTrxTypeRF215],         
    ]

    rf215_phy_at86rf_inc_files = [
    ["phy/at86rf215/inc/at86rf215.h", condTrxTypeRF215],
    ["phy/at86rf215/inc/ieee_154g.h", condTrxTypeRF215],
    ["phy/at86rf215/inc/ieee_const.h", condTrxTypeRF215],
    ["phy/at86rf215/inc/stack_config.h", condTrxTypeRF215],    
    ["phy/at86rf215/inc/tal.h", condTrxTypeRF215],
    ["phy/at86rf215/inc/tal_config.h", condTrxTypeRF215],
    ["phy/at86rf215/inc/tal_fe_fsk_params.h", condTrxTypeRF215],
    ["phy/at86rf215/inc/tal_generic.h", condTrxTypeRF215],  
    ["phy/at86rf215/inc/tal_helper_2.h", condTrxTypeRF215],  
    ["phy/at86rf215/inc/tal_internal.h", condTrxTypeRF215],  
    ["phy/at86rf215/inc/tal_multi_trx.h", condTrxTypeRF215],  
    ["phy/at86rf215/inc/tal_pib.h", condTrxTypeRF215],  
    ["phy/at86rf215/inc/tal_rf215.h", condTrxTypeRF215],
    ["phy/at86rf215/inc/tal_timer_config.h", condTrxTypeRF215],
    ["phy/at86rf215/inc/tal_types.h", condTrxTypeRF215], 
    ["phy/at86rf215/inc/tfa.h", condTrxTypeRF215],
    ["phy/at86rf215/inc/trx_access_2.h", condTrxTypeRF215],         
    ]

# #PAL layer files
#     rf215_pal_src_files = [
#     ["pal/src/pal.c"],   
#     ]

#     rf215_pal_inc_files = [
#     ["phy/inc/pal.h"],   
#     ]

# #resources files
#     rf215_bmm_src_files = [
#     ["resources/buffer/src/bmm.c"],   
#     ]

#     rf215_bmm_inc_files = [
#     ["resources/buffer/inc/bmm.h"],   
#     ]

#     rf215_qmm_src_files = [
#     ["resources/queue/src/qmm.c"],   
#     ]

#     rf215_qmm_inc_files = [
#     ["resources/queue/inc/qmm.h"],   
#     ]

# === Import the source files
    # for srcFileEntry in rf215_pal_src_files:
    #     importSrcFile(rfHostLib, configName, srcFileEntry)
    # for incFileEntry in rf215_pal_inc_files:
    #     importIncFile(rfHostLib, configName, incFileEntry)
    # for srcFileEntry in rf215_bmm_src_files:
    #     importSrcFile(rfHostLib, configName, srcFileEntry)
    # for incFileEntry in rf215_bmm_inc_files:
    #     importIncFile(rfHostLib, configName, incFileEntry)
    # for srcFileEntry in rf215_qmm_src_files:
    #     importSrcFile(rfHostLib, configName, srcFileEntry)
    # for incFileEntry in rf215_qmm_inc_files:
    #     importIncFile(rfHostLib, configName, incFileEntry)
    for srcFileEntry in rf215_phy_src_files:
        importSrcFile(rfHostLib, configName, srcFileEntry)
    print("file_list_RF215/Deepthi1",len(file_list_RF215))
    for incFileEntry in rf215_phy_inc_files:
        importIncFile(rfHostLib, configName, incFileEntry)
    print("file_list_RF215/Deepthi2",len(file_list_RF215))
    for srcFileEntry in rf215_phy_at86rf_src_files:
        importSrcFile(rfHostLib, configName, srcFileEntry)
    print("file_list_RF215/Deepthi3",len(file_list_RF215))
    for incFileEntry in rf215_phy_at86rf_inc_files:
        importIncFile(rfHostLib, configName, incFileEntry)
    print("file_list_RF215/Deepthi4",len(file_list_RF215))
    for i in range(len(file_list_RF215)):
        print(file_list_RF215[i])

def importSrcFile(component, configName, srcFileEntry, firmwarePath = None):
    srcFilePath  = srcFileEntry[0]
    isEnabled    = srcFileEntry[1][0]
    callback     = srcFileEntry[1][1]
    dependencies = srcFileEntry[1][2]

    srcFilePathTup = srcFilePath.rsplit("/", 1)

    if len(srcFilePathTup) == 1:
        secName = ""
        srcFile = srcFilePathTup[0]
    else:
        secName = srcFilePathTup[0]
        srcFile = srcFilePathTup[1]

    srcFilePrefix   = ""
    # symName = srcFile.replace(".", "_").upper()
    symName = srcFile.replace(".", "_").upper()
    secSName = secName + "/"
    secDName = secSName
    
    srcFileSym = component.createFileSymbol(symName, None)
    print("secSName",secSName)
    print("secDName",secDName)
    print("srcFile",srcFile)
    print("srcpath","driver/software/RF215/" + secSName + srcFile)
    print("destpath","driver/IEEE_802154_PHY/"+ secDName + "")
    print("project path","config/" + configName + "/"+ "driver/IEEE_802154_PHY/" + secDName + "")
    file_list_RF215.append(srcFileSym)
    srcFileSym.setSourcePath("driver/software/RF215/" + secSName + srcFile)
    srcFileSym.setOutputName(srcFile.rsplit("/", 1)[-1])
    srcFileSym.setDestPath("driver/IEEE_802154_PHY/"+ secDName + "")
    srcFileSym.setProjectPath("config/" + configName + "/"+ "driver/IEEE_802154_PHY/" + secDName + "")
    srcFileSym.setType("SOURCE")
    srcFileSym.setEnabled(isEnabled)
    srcFileSym.setOverwrite(isEnabled)

    if callback and dependencies:
        srcFileSym.setDependencies(callback, dependencies)
#end importSrcFile

def importIncFile(component, configName, incFileEntry, firmwarePath = None):
    incFilePath  = incFileEntry[0]
    isEnabled    = incFileEntry[1][0]
    callback     = incFileEntry[1][1]
    dependencies = incFileEntry[1][2]

    incFilePathTup = incFilePath.rsplit("/", 1)

    if len(incFilePathTup) == 1:
        secName = ""
        incFile = incFilePathTup[0]
        print("incFilePathTup[0]",incFilePathTup[0])
    else :
        secName = incFilePathTup[0]
        incFile = incFilePathTup[1]        

    # symName = incFile.replace(".", "_").upper()
    symName = incFile.replace(".", "_").upper()
    secSName = secName + "/"
    secDName = secSName
    print("importIncFile: ", secDName)
    print("src path")
    print("driver/software/RF215/" + secSName + incFile)
    print("dest path")
    print("driver/IEEE_802154_PHY/"+ secDName + "")
    print("proj path")
    print("config/" + configName + "/"+ "driver/IEEE_802154_PHY/" + secDName + "")
    incFileSym = component.createFileSymbol(symName, None)
    print("symName",symName)
    print("incFileSym",incFileSym)
    print("isEnabled",isEnabled)
    print("secSName",secSName.rsplit("/",2))
    file_list_RF215.append(incFileSym)
    incFileSym.setSourcePath("driver/software/RF215/" + secSName + incFile)
    incFileSym.setOutputName(incFile)
    incFileSym.setDestPath("driver/IEEE_802154_PHY/"+ secDName + "")
    incFileSym.setProjectPath("config/" + configName + "/"+ "driver/IEEE_802154_PHY/" + secDName + "")
    incFileSym.setType("HEADER")
    incFileSym.setOverwrite(True)
    incFileSym.setEnabled(isEnabled)
    incFileSym.setOverwrite(isEnabled)

    if callback and dependencies:
        incFileSym.setDependencies(callback, dependencies)
#end importIncFile

check_update_pins()
    
def finalizeComponent(rfHostLib):
    #print("finalizeComponent")
    pass
    
    
     
    