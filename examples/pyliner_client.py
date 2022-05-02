from os import path, sys
sys.path.append( path.dirname( path.dirname( path.abspath(__file__) ) ) )
from pyliner import Pyliner
from arte_client import ArteClient
import logging
import time

class PylinerClient(Pyliner):

    def __init__(self, kwargs):
        super(PylinerClient, self).__init__(**kwargs)
        self.ip = kwargs.get("ip", "localhost")
        self.port = kwargs.get("port", 9999)
        self.minor_frames = kwargs.get("minor_frames", 200)
        self.current_frame = 1
        self.total_frames = 0
        self.arte_client = ArteClient(self.ip, self.port)
   
    def step_frame(self, num = 1):
        """ Signal ARTE to step to next frame specified number of times """
        for i in range(num):
            self.arte_client.send_ready()
            self.arte_client.receive_response()
            self.current_frame = self.arte_client.sequence
            self.total_frames += 1
            
    def step_until(self, cb):
        """ Signal ARTE to step until the specified callback returns true """
        while not cb():
            self.step_frame()
            
    
    def frame_analysis(self, op, tlm, start_frame = 0):
        """ Analyze frames required to receive telemetry for this command """
        logging.info('Starting frame analysis')
        wait_flag = True
        frame_callback_var = 0
        recv_frame = 0
        sent_frame = 0
        
        # TODO: Not hard code
        def frame_analyze_cb(data):
            global frame_callback_var
            frame_callback_var = data['value']
            logging.info('Callback received value of: %s' % (frame_callback_var))
        
        # TODO: Not hard code this either
        self.send_command({'name':'/Airliner/ES/Reset'})
        self.subscribe(tlm, frame_analyze_cb)
        
        logging.info('Sending at frame: %s' % (start_frame))
        sent_flag = False
        time.sleep(0.5)
        # TODO: Loop safegaurd
        while wait_flag:
            if self.current_frame % self.minor_frames >= start_frame:
                #logging.info('current frame: %s' % (self.current_frame))
                global frame_callback_var
                time.sleep(0.02)
                
                if not sent_flag:
                    logging.info('Sending cmd at frame: %s' % (self.current_frame))
                    self.send_command(op)
                    sent_frame = self.total_frames
                    sent_flag = True
                
                if frame_callback_var == 1:
                    wait_flag = False
                    recv_frame = self.total_frames
                    
            self.step_frame()
        
        self.subscribers.pop()
        result = recv_frame - sent_frame
        logging.info('Frame analysis for %s: %s' % (op, result))
        return result
        
    def minmax_frame_analysis(self, op, tlm):
        """ Analyze frames required to receive telemetry for this command """
        logging.info('Starting min max frame analysis')
        
        best_frame_time = self.minor_frames

        for start_frame in range(1, self.minor_frames+1):
            frame_time = self.frame_analysis(op, tlm, start_frame)
            if frame_time < best_frame_time:
                best_frame_time = frame_time

        logging.info('Min max frame analysis for %s: min:%s max:%s' % (best_frame_time, self.minor_frames -1))
        return result
            



