#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#from dataclasses import dataclass
import numpy as np
import random
from offb.msg import RL_verisi
import rospy
from offb.msg import kamera_verisi


class Ucak():
    def __init__(self):
        self.Goruntu_Y = 640
        self.Goruntu_G = 480
        self.Position_y = 0  ## DEĞİŞTİRİLECEK (Yükseklik)
        self.Yere_Carpma_Esik_Degeri = 10 ## DEĞİŞTİRİLE
        self.new_data = kamera_verisi()

    def getState(self, giris_verisi):
        state = [giris_verisi[0],giris_verisi[1]]
        return np.array(state)
        


    def IsDone(self,position):   
        
        done = False
        if position <= self.Yere_Carpma_Esik_Degeri:
            done = True   

        return done
    
    


    def CalculateReward(self,state):
	first_distance = abs((self.Goruntu_Y/2)-state[0])
	second_distance = abs((self.Goruntu_G/2) -state[1])
	
	Reward = -(np.hypot(first_distance,second_distance))
        #Reward = -(np.hypot(abs(208 - state[0]), abs(208 - state[1]))) 

        #Noktanın ekranın ortasına olan linear uzaklığı (Hipotenüs)
      
	#Reward = -(((abs(208-state[0]))**2 + ((abs(208-state[1]))**2))**(1/2))
        return  Reward
        
        

    def step(self,action,data1,data2):
        ##pub.publish()


       

        done = self.IsDone(data1)
        new_state = self.getState([data1,data2])
        done = 0
        
      
        Reward = self.CalculateReward(new_state)

        return np.array(new_state) ,Reward ,done

    
   

    def RandomAction(self,max_Action):
       
        action = [max_Action * random.uniform(-1,1) , max_Action * random.uniform(-1,1),max_Action * random.uniform(-1,1) ]
        return np.array(action)


     



env_nesnesi=Ucak()
        

