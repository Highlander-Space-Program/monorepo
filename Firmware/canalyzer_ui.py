import dearpygui.dearpygui as dpg
dpg.create_context()

USB_list=[1,2,3,4,5,6,7,9]

def printValue(Sender):
    print(dpg.get_value(Sender))

def main():
    with dpg.window(label="Send Message to USB Port",width=600,height=400):
        dpg.add_text("Select USB Port")
        dpg.add_combo((1,2,3,4,5,6,7,8),callback=printValue)
        dpg.add_text("Enter message to send:")
        dpg.add_input_text(label="string", default_value="Input message")
        dpg.add_button(label="Send Message")
    

    dpg.create_viewport(title="Send Message", width=1000,height=600)
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.start_dearpygui()
    dpg.destroy_context()

if __name__ == "__main__":
    main()
