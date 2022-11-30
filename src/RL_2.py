#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# from distutils.log import info
import rospy
from buffer import loadReplay,saveReplay
from env import Ucak
from td3 import TD3
from td3 import ReplayBuffer
import rospy
import time
import torch
import numpy as np
from offb.msg import offboard
from offb.msg import kamera_verisi
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Quaternion
from geometry_msgs.msg import Vector3

# RL içinde yapay_zeka topiciğine bağlı isplane değişkenine göre yapay zeka RL çalıştır, çıktılarıda offboard topiciğine yolla.
# thrust value, roll value pitch value, //yaw value kabul etmiyor.
# float float float
# 0-1  +-40 +-40
# https://mavsdk.mavlink.io/main/en/cpp/api_reference/structmavsdk_1_1_offboard_1_1_attitude.html
from tf.transformations import quaternion_from_euler

from mavros_msgs.msg import AttitudeTarget


class RL():
    def __init__(self):
       
        self.step = 10501 ## Burası her eğitim başlatıldığında kaçıncı adımda kalındıysa o sayıyla güncellenmeli
        state_dim = 2
        action_dim = 3
        self.max_action = 1
        self.total_timesteps = 100000
        self.start_timesteps = 10000
        self.buffer = ReplayBuffer()
        self.td3 = TD3(state_dim, action_dim, self.max_action)
        self.env = Ucak()
        self.giris_verisi = [0, 0, 0]
        loadReplay("/home/burak/catkin_ws/src/offb/src/RLağırlık",self.buffer) # Eğitimi her başlattığımda buffer'daki verilerin 0'lanmaması için alınan verileri replay.txt dosyasına kaydettim.
        #Her eğitim başlatıldığında loadReplay fonksiyonu txt dosyasındaki verileri okuyup buffer'a aktarıyor. (buffer.py)
        self.att = AttitudeTarget()
        self.att_setpoint_pub = rospy.Publisher('uav0/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)


        try:
            self.td3.load("ağırlıklar_10500", "/home/burak/catkin_ws/src/offb/src/RLağırlık2") # Burdaki ağırlıklar da eğitimde en son hangi ağırlıkta kalındıysa onla güncellenmeli
	    print("Mevcut Ağırlıklar yüklendi")
	    
        except:
            print("Ağirliklar yüklenemedi")
	
	
	time.sleep(1)
	print("Görüntü bekleniyor...")
    def TakeAction(self):
        if ((self.step % 10) == 0) and (self.step != 0):
            print("Eğitim başlatiliyor")
            print("***********************")
	    print("\n \n \n \n \n \n \n")
            self.td3.train(replay_buffer=self.buffer, iterations=5) #İterations kısmına normalde bir episode'da atılan adım sayısı yazılması gerek
#fakat episode'lu çalışamadığımız için doğru sayıyı bulamadım
	    
	    
	    print("*************************************")
            print("Eğitim Tamamlandı")

	    if (self.step % 50 == 0):
	    	
		    print("Ağirliklar Kaydediliyor")
		    print("*************************************")
		    print("\n \n \n \n \n \n \n")
		    self.td3.save("ağırlıklar_{}".format(self.step), "/home/burak/catkin_ws/src/offb/src/RLağırlık2")
		    
		    print("**********************")
                    print("Ağirliklar Kaydedildi")
		    self.step +=1
		    
		    return
	    else:
		    self.step+=1

        
        #print(self.giris_verisi)
        state = self.env.getState([self.giris_verisi.x_yatay, self.giris_verisi.y_dikey])
        

        if self.giris_verisi.isPlane == True:
            
           
            if self.total_timesteps < self.start_timesteps:
                action = torch.Tensor(self.env.RandomAction(self.max_action)).cpu().data.numpy().flatten()
	        action_for_buffer = np.copy(action) # buffer'a işlenmemiş (direkt RL'den çıkan) aksiyon değerleini göndermek istedim fakat doğru yöntem mi emin değilim. Buffer training sırasında kullanılıyor
                # Thrust'ı 0 ve 1 arasına getirme
                thrustarrayy = np.array([-1,action[0],1])
		thrustarrayy2 = (thrustarrayy-np.min(thrustarrayy))/np.ptp(thrustarrayy)
                # Thrust'ı 0 ve 1 arasına getirme
		action[0] = thrustarrayy2[1]
		action[1] = action[1] * 30
		action[2] = action[2] * 30
                

            else:
                action = (self.td3.select_action(state)+np.random.normal(0, self.max_action * 0.1,size = 3)).clip(-1,1) #Rl'in seçtiği aksiyonlar -1 ile 1 arasına getiriliyor
	        action_for_buffer = np.copy(action)
		thrustarray = np.array([-1,action[0],1])
		thrustarray2 = (thrustarray-np.min(thrustarray))/np.ptp(thrustarray)
		action[0] = thrustarray2[1]
		action[1] = action[1] * 30
		action[2] = action[2] * 30
	        

            
            self.offboard_yayinlayici(action)  # offboard a publish eder
       
            #time.sleep(0.01)
            yeni_veri=rospy.wait_for_message('/kamera_verisi', kamera_verisi)
	    if (yeni_veri.isPlane == True):
		    self.step += 1
		    print("X ve Y noktalari:[{:.0f},{:.0f}]".format(state[0],state[1]))

	            #print("Bu eğitimdeki step sayisi:{}".format(self.step))
		    print("Aksiyon Değerleri:[{:.3f},{:.2f},{:.2f}]".format(action[0],action[1],action[2]))
                    print("Ham Aksiyon Değerleri:[{:.3f},{:.2f},{:.2f}]".format(action_for_buffer[0],action_for_buffer[1],action_for_buffer[2]))
		    yeni_state = np.array([yeni_veri.x_yatay,yeni_veri.y_dikey])
		    rew1 = -(self.env.CalculateReward(state))
		    rew2 = -(self.env.CalculateReward(yeni_state))
		    #print("reward 1 : {:.1f} , reward2 : {:.1f}".format(rew1,rew2))
           

		    (new_state, reward, done) = self.env.step(action,yeni_veri.x_yatay,yeni_veri.y_dikey)
		    reward = rew1 - rew2
		    saveReplay("/home/burak/catkin_ws/src/offb/src/RLağırlık",(state,new_state,action_for_buffer,reward,done),self.buffer) # replay.txt dosyasına kayıt (buffer.py)
		    self.buffer.add((state, new_state, action_for_buffer, reward, done))

		    print("Toplam adim: {} , Bu adimda alinan ödül: {:.2f}".format(self.step, reward))
		    return done
	    else:
		    return 

        

    def callback(self):
        self.giris_verisi = rospy.wait_for_message('/kamera_verisi', kamera_verisi)

        if self.giris_verisi.isPlane==True:
            
            self.TakeAction()
	

    def offboard_yayinlayici(self, data):
        rate = rospy.Rate(10)  # Hz
        self.att.header = Header()
        self.att.header.frame_id = "base_footprint"
        self.att.body_rate = Vector3()  # yaw pitch roll vector3

        #self.att.orientation = Quaternion(*quaternion_from_euler(data[[0]], data[[1]], 0))

        self.att.thrust = data[0]
	self.att.body_rate.x = data[1]
	self.att.body_rate.y = data[2]

        self.att.type_mask = 132  # ignore body rate.z(yaw)

        """
        while not rospy.is_shutdown():
            self.att.header.stamp = rospy.Time.now()
            self.att_setpoint_pub.publish(self.att)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
		print("ROsinterruptshutdown")
                pass        

	"""
        self.att.header.stamp = rospy.Time.now()
        self.att_setpoint_pub.publish(self.att)



rospy.init_node('RL_node_1', anonymous=True)
RL_object = RL()
#rospy.Subscriber("/kamera_verisi", kamera_verisi, RL_object.callback)


if __name__ == '__main__':
    try:
        while True:
            RL_object.callback()
    except rospy.ROSInterruptException:
        pass
