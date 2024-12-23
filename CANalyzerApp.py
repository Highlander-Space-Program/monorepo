import dearpygui.dearpygui as dpg
from screeninfo import get_monitors

dpg.create_context()
dpg.create_viewport(title = "Dropdown", width = 600, height = 600)

def printVal(Sender):
    print(dpg.get_value(Sender))

def get_screen_resolution():
    monitor = get_monitors()[0]
    return monitor.width, monitor.height

screenWidth, screenHeight = get_screen_resolution()
    
with dpg.window(label = "Dropdown", width=screenWidth, height=screenHeight):
    dpg.add_text("Select Port")
    dpg.add_combo((1,2,3,4,5), callback=printVal)
    dpg.add_input_text(label = "string", default_value = "message")
    dpg.add_button(label = "Send")
        
dpg.setup_dearpygui()
dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()
