# Interface to communicate with Sol
# Version 07-06-2021 

import ac

debug_label = 0

def interface_set_dbg_label(label):
    global debug_label
    debug_label = label

def dbg(msg):
    global debug_label
    ac.setText(debug_label, msg)

class SolInterface:

    def __init__(self, appID):
        self.in_counter = 0
        self.out_counter = 0
        self.send_buffer = []
        self.appID = appID
        self.order_executor_dict = {}
        self.running = False

        self.stop()

    def is_digit(self, str):
        return str.isdigit()

    def split_string(self, s, sep):  
        return s.split(sep)

    def set_order_executor(self, dict):
        if dict:
            self.order_executor_dict = dict

    def restart(self):
        self.in_counter = 0
        self.out_counter = 0

    def check_order(self, parts):
        order = {}

        if parts[0] and len(parts)>0:
            if 'SET_VALUE' in parts[0] or 'INIT_VALUE' in parts[0]:
                if parts[1] and parts[2] and parts[3]:
                    
                    order={'type':parts[0],
                        'section':parts[1],
                        'key':parts[2],
                        'value':0,
                        'relative':False}
                
                    if "true" in parts[3]:
                        order['value'] = True
                    elif "false" in parts[3]:
                        order['value'] = False
                    else:
                        order['value'] = float(parts[3])

                    if len(parts)>4:
                        if "relative" in parts[4]:
                            order['relative'] = True

            if 'CMD' in parts[0]:
                order={'type':parts[0]}
                for i in range(1, len(parts)):
                    order['CMD%i'%i] = parts[i]
                    
        return order

    def generate_orders(self, commandos):
        orders = []
        
        if commandos and len(commandos)>0:
            for v in commandos:  
                msg_parts = self.split_string(v, ":")
                if msg_parts and len(msg_parts)>0:
                    tmp = self.check_order(msg_parts)
                    if tmp and len(tmp)>0:
                        orders.append(tmp)
            
        return orders

    def get_commandos(self, msg):
        commandos = []

        msg_commandos = self.split_string(msg, "#")
        if msg_commandos and len(msg_commandos)>0:
            for v in msg_commandos:
                if len(v)>0:
                    commando = self.split_string(v, "$")
                    if commando and len(commando)>0 and self.is_digit(commando[0]):
                        new_counter = int(commando[0])
                        #ac.log('%i,%i'%(new_counter, self.in_counter))
                        if new_counter>self.in_counter or new_counter==0:
                            self.in_counter = new_counter
                            commando.pop(0) #remove the counter entry from the list
                            commandos.extend(commando)

        return commandos

    def execute_orders(self, orders):

        if orders and len(orders)>0:
            for v in orders: #go through all orders
                if v['type']:
                    if self.order_executor_dict and self.order_executor_dict[v['type']]:
                        self.order_executor_dict[v['type']](v)

        return True


    def receive_from_Sol(self):

        if self.appID and len(self.appID)>0:
            msg = ac.ext_loadLua('sol.TO_'+self.appID) 
            #ac.log('%s'%msg)
            if msg and len(msg)>0:
                cmds = self.get_commandos(msg)
                #ac.log('%i'%len(cmds))
                if cmds and len(cmds)>0:
                    orders = self.generate_orders(cmds)
                    if orders and len(orders)>0:
                        self.execute_orders(orders)

        # tell its ready to receive the next data
        ac.ext_storeLua('sol.TO_'+self.appID+'_CTS', "OK")

        # clear the data
        ac.ext_storeLua('sol.TO_'+self.appID, '')

        return True


    def set_value(self, section, key, value):
        bAlreadyEntry = False

        for v in self.send_buffer: 
            if 'SET_VALUE' in v['type'] and 'Section' in v and 'Key' in v:
                if section in v['Section'] and key in v['Key']:
                    v['value'] = value
                    if bAlreadyEntry:
                        ac.log("SolCom.set_value() - doubled entry:%s,%s"%(section, key))
                    bAlreadyEntry = True
                    #break
        
        if not bAlreadyEntry:
            self.send_buffer.append({'type':'SET_VALUE', 'Section':section, 'Key':key, 'value':value})
        #else:
        #    ac.log("SolCom.set_value() - updated:%s,%s"%(section, key))

        return True

    def init_value(self, section, key, value):
        bAlreadyEntry = False

        for v in self.send_buffer: 
            if 'INIT_VALUE' in v['type'] and 'Section' in v and 'Key' in v:
                if section in v['Section'] and key in v['Key']:
                    v['value'] = value
                    bAlreadyEntry = True
                    break
        
        if not bAlreadyEntry:
            self.send_buffer.append({'type':'INIT_VALUE', 'Section':section, 'Key':key, 'value':value})

        return True


    def send_command(self, args):
        bAlreadyEntry = False

        if args:
            n_args = min(10, len(args)) #limit amount of commands to 10
            n_curr = 1
            if n_args>0:
                for v in self.send_buffer: # check if command is already in send buffer
                    if 'CMD' in v['type']:
                        found = False
                        cmd = 'CMD'
                        while n_curr<n_args:
                            cmdn = cmd+('%i'%n_curr) # 'CMD1...9'
                            if v[cmdn] and v[cmdn] in args[n_curr-1]:
                                found = True
                            else:
                                found = False
                                break
                            n_curr += 1

                        if found == True:
                            cmdn = cmd+('%i'%n_args)
                            if v[cmdn]:
                                v[cmdn] = args[n_args-1]

                            bAlreadyEntry = True

                            #msg = ""
                            #for tmp in v:
                            #    msg += "%s=%s,"%(tmp,str(v[tmp]))
                            #ac.log("SolCom.send_command() - updated:%s"%(msg))
                            
                            break

                if not bAlreadyEntry:
                    t = {'type':'CMD'}
                    n_curr = 1
                    cmd = 'CMD'
                    for v in args:
                        cmdn = cmd+('%i'%n_curr)
                        t[cmdn] = v
                        n_curr += 1
                    
                    self.send_buffer.append(t)

                    
    def send(self, msg):
        if self.appID and len(self.appID)>0:
            if msg and len(msg)>0:
                ac.ext_storeLua('sol.FROM_'+self.appID, msg)
                self.out_counter += 1
                if self.out_counter > 10000:
                    self.out_counter = 0

    def checkCTS(self): # check if receiver is Clear To Send
        if self.appID and len(self.appID)>0:
            msg = ac.ext_loadLua('sol.FROM_'+self.appID+'_CTS')
            if msg and len(msg)>0 and 'OK' in msg:
                return True
            else:
                return False

    def send_to_Sol(self):
        if self.checkCTS() == True: # if receiver is Clear To Send
            if self.send_buffer and len(self.send_buffer)>0:

                msg = "#" + ('%i'%self.out_counter)

                for v in self.send_buffer:
                    if v and len(v)>0:
                        if 'SET_VALUE' in v['type'] and 'Section' in v and 'Key' in v:
                            msg += '$SET_VALUE:' + v['Section'] + ':' + v['Key'] + ':'
                            if type(v['value']) is bool:
                                if v['value'] == True:
                                    msg += 'true'
                                else:
                                    msg += 'false'
                            else:
                                msg += ('%.3f'%v['value'])
                        elif 'INIT_VALUE' in v['type'] and 'Section' in v and 'Key' in v:
                            msg += '$INIT_VALUE:' + v['Section'] + ':' + v['Key'] + ':'
                            if type(v['value']) is bool:
                                if v['value'] == True:
                                    msg += 'true'
                                else:
                                    msg += 'false'
                            else:
                                msg += ('%.3f'%v['value'])
                        elif 'CMD' in v['type']:
                            cmd = 'CMD'
                            msg += '$CMD'
                            n_cmds = min(10, len(v)) # do not sub 1, if we use "<". We already start with 1
                            n_curr = 1
                            while n_curr<n_cmds:
                                cmdn = cmd+('%i'%n_curr) # 'CMD1...9'
                                if v[cmdn]:
                                    msg += ':'+v[cmdn]
                                n_curr += 1

                self.send(msg)

                self.send_buffer = [] #clear the send buffer

                # clear CTS state
                ac.ext_storeLua('sol.FROM_'+self.appID+'_CTS', "WAITING")

        return True


    def update(self):

        if self.running:
            self.receive_from_Sol()
            self.send_to_Sol()
            return True
        else:
            return False

    def run(self):
        if not self.running:
            self.running = True
            ac.ext_storeLua('sol.TO_'+self.appID+'_CTS', "OK")

    def stop(self):
        self.running = False
        self.in_counter = 0
        self.out_counter = 0
        self.send_buffer = [] #clear the send buffer
        ac.ext_storeLua('sol.TO_'+self.appID+'_CTS', "SLEEPING")

    def close(self):
        return True

    def __del__(self):
        self.close()
            