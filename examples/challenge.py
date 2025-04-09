import pyliner
import time
from arte_client import ArteClient

run_flag = True
cmd_count = 0
recv_frame = 0
sent_frame = 0

def cmd_count_callback(data):
    global cmd_count
    cmd_count = data['params']['CmdCounter']['value']
    print "Cmd counter: " + str(cmd_count)
    recv_frame = client.sequence
    print "Cmd processed in %s frames" % (recv_frame - sent_frame)

airliner = pyliner.Pyliner(**{"airliner_map": "cookiecutter.json", "test_name": "demo_challenge"})
airliner.subscribe({'name': '/Airliner/ES/HK', 'args':[{'name':'CmdCounter'}]}, cmd_count_callback)

IP, PORT = "localhost", 9999
client = ArteClient(IP, PORT)

while run_flag:
    #time.sleep(0.001)
    
    client.send_ready()
    client.receive_response()
    
    if client.sequence % 200 == 0:
        print"sending no-op"
        airliner.send_message({'name': '/Airliner/ES/Noop'})
        sent_frame = client.sequence
    
    if cmd_count == 400:
        run_flag = False
airliner.finish_test()
client.send_shutdown(True)
client.close_conn()
        



