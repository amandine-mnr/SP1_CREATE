import LCD_1in44

import time

from PIL import Image,ImageDraw,ImageFont,ImageColor

menu_items = ["Destination A", "Destination B", "Destination C",
                "Destination D", "Destination E", "Destination F",
                "Destination G", "Destination H"]
MAX_VISIBLE = 5 #nb of items visible on the screen
scroll_offset = 0 #first visible item
old_button = None

display_color = "purple"

disp = LCD_1in44.LCD()
Lcd_ScanDir = LCD_1in44.SCAN_DIR_DFT
disp.LCD_Init(Lcd_ScanDir)
disp.LCD_Clear()

# Create blank image for drawing.
image = Image.new('RGB', (disp.width, disp.height))
draw = ImageDraw.Draw(image)
draw.rectangle((0,0,disp.width,disp.height), outline=0, fill=0)
disp.LCD_ShowImage(image,0,0)

#############

def get_button():
    button = None
    if not((disp.digital_read(disp.GPIO_KEY_UP_PIN)) == 0): # button is pressed
        button = "up"    

    if not(disp.digital_read(disp.GPIO_KEY_LEFT_PIN) == 0): # button is pressed 
        button = "left"

    if not(disp.digital_read(disp.GPIO_KEY_RIGHT_PIN) == 0): #button is pressed     
        button = "right"

    if not(disp.digital_read(disp.GPIO_KEY_DOWN_PIN) == 0): #button is pressed 
        button = "down"

    if not(disp.digital_read(disp.GPIO_KEY_PRESS_PIN) == 0): #button is pressed
        button = "center"
    
    return button

def update_scroll(selected, total):
    global scroll_offset

    if selected < scroll_offset:
        scroll_offset = selected
    elif selected >= scroll_offset + MAX_VISIBLE:
        scroll_offset = selected - MAX_VISIBLE + 1

def draw_menu(selected_index):
    global image, draw, scroll_offset
    font = ImageFont.load_default()
    image = Image.new("RGB", (disp.width, disp.height), "black")
    draw = ImageDraw.Draw(image)

    visible_items = menu_items[scroll_offset : scroll_offset + MAX_VISIBLE]

    y = 10
    for i, text in enumerate(visible_items):
        real_index = i + scroll_offset

        if real_index == selected_index:
            draw.rectangle((0, y - 2, disp.width, y + 14), fill="mediumpurple")
            draw.text((5, y), text, font=font, fill="white")
        else:
            draw.text((5, y), text, font=font, fill=display_color)
            
        y += 20

    draw_scroll_indicator(draw, len(menu_items), scroll_offset)
    disp.LCD_ShowImage(image, 0, 0)

def draw_moving_menu(selected_index):
    global image, draw, scroll_offset

    font = ImageFont.load_default()
    image = Image.new("RGB", (disp.width, disp.height), "black")
    draw = ImageDraw.Draw(image)

    menu_items_moving = ["Stop"] + menu_items

    visible_items = menu_items_moving[scroll_offset : scroll_offset + MAX_VISIBLE]

    y = 10
    for i, text in enumerate(visible_items):
        real_index = i + scroll_offset

        if real_index == selected_index:
            if real_index == 0 :
                draw.rectangle((0, y - 2, disp.width, y + 14), fill="seashell")
                draw.text((5, y), text, font=font, fill="crimson")
            else :
                draw.rectangle((0, y - 2, disp.width, y + 14), fill="mediumpurple")
                draw.text((5, y), text, font=font, fill="white")
        else:
            if real_index == 0 :
                draw.text((5, y), text, font=font, fill="crimson")
            else :
                draw.text((5, y), text, font=font, fill=display_color)

        y += 20
    
    draw_scroll_indicator(draw, len(menu_items), scroll_offset)
    disp.LCD_ShowImage(image, 0, 0)

def draw_scroll_indicator(draw, total, scroll_offset):
    if total <= MAX_VISIBLE:
        return

    bar_x1 = disp.width - 6
    bar_x2 = disp.width - 3
    bar_y1 = 5
    bar_y2 = disp.height - 5

    draw.rectangle((bar_x1, bar_y1, bar_x2, bar_y2), fill=(60,60,60)) #scroll track

    #Thumb
    track_height = bar_y2 - bar_y1
    thumb_height = max(10, int(track_height * (MAX_VISIBLE / total)))
    max_scroll = total - MAX_VISIBLE
    thumb_y = bar_y1 + int((track_height - thumb_height) * (scroll_offset / max_scroll))
    draw.rectangle((bar_x1, thumb_y, bar_x2, thumb_y + thumb_height), fill="crimson")


######################
NB_DEST = len(menu_items)
selected = 0  #index of selected item
state = "WAITING"
repeat_counter = 0

while True:
    button = get_button()

    if state == "WAITING":
        if (button is not None) :
            if (button != old_button):
                repeat_counter = 0
                print(button)
                if(button == "up"):
                    selected -= 1
                if(button == "down"):
                    selected += 1
                if(button == "center"):
                    selected = 0
                    scroll_offset = 0
                    state = "MOVING"
            else:
                repeat_counter += 1
            if (repeat_counter>4):
                repeat_counter = repeat_counter % 4
                if(button == "up"):
                    selected -= 1
                if(button == "down"):
                    selected += 1

        selected = selected % NB_DEST
        update_scroll(selected, NB_DEST)
        draw_menu(selected)

    elif state == "MOVING":
        total_items = NB_DEST + 1

        if (button is not None):
            if (button != old_button):
                print(button)
                if(button == "up"):
                    selected -= 1
                if(button == "down"):
                    selected += 1
                if(button == "center"):
                    if(selected == 0):
                        state = "WAITING"
                    selected = 0
                    scroll_offset = 0
            else:
                repeat_counter += 1
                if (repeat_counter>4):
                    repeat_counter = repeat_counter % 4
                    if(button == "up"):
                        selected -= 1
                    if(button == "down"):
                        selected += 1

        selected = selected % total_items
        update_scroll(selected, total_items)
        draw_moving_menu(selected)

    old_button = button
    time.sleep(0.05)

disp.module_exit()
