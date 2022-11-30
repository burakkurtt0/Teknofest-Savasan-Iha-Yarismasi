from argparse import Action
from cv2 import line
from td3 import ReplayBuffer
import numpy as np
from ast import literal_eval

def loadReplay(path,buff):
    f = open("{}/replay.txt".format(path),"r")
    for i in f:
        try:
            linelist = i.split(":")
            
            statestring = literal_eval(linelist[0])          
            statestring = np.array(statestring)
            state = statestring.astype(np.float64)
            

            #new_state = np.array(linelist[1])
            new_state_String = literal_eval(linelist[1])
            new_state_String = np.array(new_state_String)
            new_state = new_state_String.astype(np.float64)


            #action = np.array(linelist[2])
            action_string = literal_eval(linelist[2])
            action_string = np.array(action_string)
            action = action_string.astype(np.float64)
            

            reward = linelist[3]
            reward = np.array(reward)
            reward = reward.astype(np.float64)
            
            

            done = linelist[4]
            done = np.array(done)
            done = done.astype(np.float64)
            
           
            buff.add((state,new_state,action,reward,done))

        except:
            print("hatali satir")

    print("Replaybuffer hazir")

def saveReplay(path,transition,buff):
    f = open("{}/replay.txt".format(path),"a")
    f.write("{}:{}:{}:{}:{}:\n".format(list(transition[0]),list(transition[1]),list(transition[2]),transition[3],transition[4]))
    print("veriler kaydedildi")
    f.close()
