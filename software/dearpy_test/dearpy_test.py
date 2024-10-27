import dearpygui.dearpygui as dpg
import pyautogui
import random
from multiprocessing import Process, Queue
import time

def close_button(sender, app_data, user_data):
    dpg.destroy_context()

def close_window_button(sender, app_data, user_data):
    dpg.delete_item(f"{user_data}")

def set_text(text_id, text):
    dpg.set_value(text_id, f"{text}")

def button_callback(sender, app_data, user_data):
    dpg.create_context()
    with dpg.window(label="child window", tag="Secondary Window"):
        dpg.add_button(label="YEET", parent="Secondary Window", callback=close_window_button, user_data="Secondary Window")
        dpg.add_text("starter text", tag="text2")
    #dpg.setup_dearpygui()
    #dpg.show_viewport()
    #dpg.start_dearpygui()
    #dpg.destroy_context()

def generate_number(queue):
    while 1:
        num = random.random()
        queue.put(num)
        time.sleep(0.1)

if __name__ == '__main__':
    dpg.create_context()
    size = pyautogui.size()
    dpg.create_viewport(title='Custom Title', width=size[0], height=size[1])

    q = Queue()
    p = Process(target=generate_number, args=(q,))
    p.daemon=True
    p.start()

    with dpg.window(tag="Primary Window", label="Example Window"):
        dpg.add_text("Hello, world")
        dpg.add_button(label="Save")
        dpg.add_input_text(label="string", default_value="Quick brown fox")
        dpg.add_slider_float(label="float", default_value=0.273, max_value=1)
        with dpg.group():
            le_button = dpg.add_button(label="le button", callback=button_callback, user_data="user data")
            le_button2 = dpg.add_button(label="CLOSE", callback=close_button)
            with dpg.group() as group1:
                pass
        dpg.add_text("starter text", tag="text")

    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.set_primary_window("Primary Window", True)
    while dpg.is_dearpygui_running():
        if not q.empty():
            val = q.get()
            set_text("text", val)
            if dpg.get_alias_id("Secondary Window") in dpg.get_windows():
                set_text("text2", val)
        dpg.render_dearpygui_frame()
    dpg.destroy_context()
    p.terminate()