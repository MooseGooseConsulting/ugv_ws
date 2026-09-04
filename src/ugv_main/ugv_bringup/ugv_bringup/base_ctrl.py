import serial
import json
import queue
import threading
import logging
import time

class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s
        self.timeout = 0.1 
        self.frame_start = b'{'
        self.frame_end =  b"}\r\n"
        self.max_frame_length = 512
 
    def readline(self):
        start_time = time.time() 
        while True:
            i = max(1, min(self.max_frame_length, self.s.in_waiting))
            data = self.s.read(i)
            if data:
                self.buf.extend(data)

            end = self.buf.rfind(self.frame_end)

            if end >= 0:  
                start = self.buf.rfind(self.frame_start, 0, end)
                if start >= 0 and start < end:
                    r = self.buf[start:end + len(self.frame_end)] 
                    self.buf = self.buf[end + len(self.frame_end):]
                    return r
                elif start == -1:
                    continue

            if time.time() - start_time > self.timeout:
                break
 
    def clear_buffer(self):
        self.buf = bytearray()
        self.last_complete_line = bytearray()
        try:
            self.s.reset_input_buffer()
        except Exception as e:
            print(f"Error resetting input buffer: {e}")
 
    def has_last_complete_line(self):
        return len(self.last_complete_line) > 0
 
    def get_last_complete_line(self):
        return bytes(self.last_complete_line)
                
class BaseController:
    def __init__(self, uart_dev_set, baud_set):
        self.log = logging.getLogger('BaseController')
        self.ser = serial.Serial(uart_dev_set, baud_set, timeout=0.01, exclusive=True)  
        self.rl = ReadLine(self.ser)
        self.command_dict = {}
        self.lock = threading.Lock()
        self.running = True
        self.command_thread = threading.Thread(target=self.process_commands, daemon=True)
        self.command_thread.start()
        self.data_buffer = None
        self.read_fail_count = 0 
        self.read_fail_threshold = 5

        self.base_data = {"T": 1001, "L": 0, "R": 0, "ax": 0, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0, "mx": 0, "my": 0, "mz": 0, "odl": 0, "odr": 0, "v": 0}
        
    def feedback_data(self):
        try:
            line = self.rl.readline()
            if not line:
                self.read_fail_count += 1
                if self.read_fail_count >= self.read_fail_threshold:
                    self.rl.clear_buffer()
                    self.read_fail_count = 0
                return

            line = line.decode('utf-8')
            self.data_buffer = json.loads(line)
            self.base_data = self.data_buffer
            self.read_fail_count = 0  
            return self.base_data

        except json.JSONDecodeError as e:
            self.log.error(f"JSON decode error: {e} with line: {line}")
            self.read_fail_count += 1
            if self.read_fail_count >= self.read_fail_threshold:
                self.rl.clear_buffer()
                self.read_fail_count = 0

        except Exception as e:
            self.log.error(f"[base_ctrl.feedback_data] unexpected error: {e}")
            self.read_fail_count += 1
            if self.read_fail_count >= self.read_fail_threshold:
                self.rl.clear_buffer()
                self.read_fail_count = 0

    def on_data_received(self):
        self.ser.reset_input_buffer()
        data_read = json.loads(self.rl.readline().decode('utf-8'))
        return data_read
        
    def send_command(self, data: bytes):
        try:
            json_data = json.loads(data.decode())
            t = json_data.get("T")
            if t is not None:
                if str(t) == "3" or str(t) == "13":
                    try:
                        self.ser.write(data)
                    except Exception as e:
                        print(f"Failed to immediately send T=3 data: {e}")
                    return
                with self.lock:
                    self.command_dict[str(t)] = data  
        except Exception as e:
            print(f"Failed to parse or store command: {e}")

    # Thread function to process and send commands from the queue
    def process_commands(self):
        while self.running:
            time.sleep(0.01)  
            with self.lock:
                if not self.command_dict:
                    continue
                for t_key, data in self.command_dict.items():
                    try:
                        self.ser.write(data)
                    except Exception as e:
                        print(f"Failed to send {t_key}: {e}")
                self.command_dict.clear()  

    def close(self):
        self.running = False
        self.ser.close()