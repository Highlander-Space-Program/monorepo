import dearpygui.dearpygui as dpg
from screeninfo import get_monitors

dpg.create_context()

def printValue(Sender):
    print(dpg.get_value(Sender))

def get_screen_resolution():
    monitor = get_monitors()[0]
    return monitor.width, monitor.height

def main():
    s_width, s_height = get_screen_resolution()
    
    with dpg.window(label="Send Message to USB Port",width=s_width, height=s_height):
        dpg.add_text("Select USB Port")
        dpg.add_combo((1,2,3,4,5,6,7,8),callback=printValue)
        dpg.add_text("Enter message to send:")
        dpg.add_input_text(label="string", default_value="Input message")
        dpg.add_button(label="Send Message")
    

    dpg.create_viewport(title="Send Message", width=s_width, height=s_height)
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.start_dearpygui()
    dpg.destroy_context()

if __name__ == "__main__":
    main()
