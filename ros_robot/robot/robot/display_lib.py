import time
from PIL import Image, ImageDraw, ImageFont
from robot import LCD_1in44

# States
DISP_WAIT = 200
DISP_MOVE = 201

class LCDMenu:
    MAX_VISIBLE = 5 #nb of items visible on the screen

    def __init__(self, menu_items):
        self.menu_items = menu_items
        self.total = len(menu_items)

        # Menu state
        self.selected = 0
        self.scroll_offset = 0
        self.last_button = None
        self.repeat_counter = 0

        # Current mode : DISP_WAIT or DISP_MOVE
        self.mode = DISP_WAIT

        # Returned command
        self.command = None

        # Setup display
        self.disp = LCD_1in44.LCD()
        self.disp.LCD_Init(LCD_1in44.SCAN_DIR_DFT)
        self.disp.LCD_Clear()

        self.image = Image.new("RGB", (self.disp.width, self.disp.height), "black")
        self.draw = ImageDraw.Draw(self.image)
        self.font = ImageFont.load_default()
        self.color = "purple"

    # Button reading
    def get_button(self):
        disp = self.disp

        if not((disp.digital_read(disp.GPIO_KEY_UP_PIN)) == 0):
            return "up"
        if not(disp.digital_read(disp.GPIO_KEY_LEFT_PIN) == 0):
            return "left"
        if not(disp.digital_read(disp.GPIO_KEY_RIGHT_PIN) == 0):
            return "right"
        if not(disp.digital_read(disp.GPIO_KEY_DOWN_PIN) == 0):
            return "down"
        if not(disp.digital_read(disp.GPIO_KEY_PRESS_PIN) == 0):
            return "center"

        return None

    # Scroll

    def update_scroll(self, visible_count):
        if self.selected < self.scroll_offset:
            self.scroll_offset = self.selected
        elif self.selected >= self.scroll_offset + visible_count:
            self.scroll_offset = self.selected - visible_count + 1

    def draw_scroll_indicator(self, total):
        if total <= self.MAX_VISIBLE:
            return

        bar_x1 = self.disp.width - 6
        bar_x2 = self.disp.width - 3
        bar_y1 = 5
        bar_y2 = self.disp.height - 5

        track_h = bar_y2 - bar_y1
        thumb_h = max(10, int(track_h * (self.MAX_VISIBLE / total)))

        max_scroll = total - self.MAX_VISIBLE
        thumb_y = bar_y1 + int((track_h - thumb_h) * (self.scroll_offset / max_scroll))

        self.draw.rectangle((bar_x1, bar_y1, bar_x2, bar_y2), fill=(60, 60, 60)) #background track
        self.draw.rectangle((bar_x1, thumb_y, bar_x2, thumb_y + thumb_h), fill="crimson") #thumb

    def draw_menu_waiting(self): #display only destinations
        self.image = Image.new("RGB", (self.disp.width, self.disp.height), "black")
        self.draw = ImageDraw.Draw(self.image)

        visible = self.menu_items[self.scroll_offset: self.scroll_offset + self.MAX_VISIBLE]
        y = 10

        for i, text in enumerate(visible):
            idx = self.scroll_offset + i

            if idx == self.selected:
                self.draw.rectangle((0, y - 2, self.disp.width, y + 14), fill="mediumpurple")
                self.draw.text((5, y), text, font=self.font, fill="white")
            else:
                self.draw.text((5, y), text, font=self.font, fill=self.color)

            y += 20

        self.draw_scroll_indicator(self.total)
        self.disp.LCD_ShowImage(self.image, 0, 0)

    def draw_menu_moving(self): #display Stop + destinations
        
        items = ["Stop"] + self.menu_items
        total = len(items)

        self.image = Image.new("RGB", (self.disp.width, self.disp.height), "black")
        self.draw = ImageDraw.Draw(self.image)

        visible = items[self.scroll_offset: self.scroll_offset + self.MAX_VISIBLE]
        y = 10

        for i, text in enumerate(visible):
            idx = self.scroll_offset + i

            if idx == self.selected:
                if idx == 0:
                    self.draw.rectangle((0, y - 2, self.disp.width, y + 14), fill="seashell")
                    self.draw.text((5, y), text, font=self.font, fill="crimson")
                else:
                    self.draw.rectangle((0, y - 2, self.disp.width, y + 14), fill="mediumpurple")
                    self.draw.text((5, y), text, font=self.font, fill="white")
            else:
                if idx == 0:
                    self.draw.text((5, y), text, font=self.font, fill="crimson")
                else:
                    self.draw.text((5, y), text, font=self.font, fill=self.color)

            y += 20

        self.draw_scroll_indicator(total)
        self.disp.LCD_ShowImage(self.image, 0, 0)

    # Mode handling

    def set_mode(self, mode):
        if mode != self.mode: #mode was changed by the FSM
            self.mode = mode
            self.selected = 0
            self.scroll_offset = 0

    def handle_waiting(self, button):
        if button:
            if button != self.last_button:
                self.repeat_counter = 0

                if button == "up": self.selected -= 1
                if button == "down": self.selected += 1
                if button == "center":
                    dest = self.menu_items[self.selected]
                    self.command = dest
                    print("Selected:", dest)
                    self.selected = 0
                    return

            else:
                self.repeat_counter += 1
                if self.repeat_counter > 4:
                    self.repeat_counter %= 4
                    if button == "up": self.selected -= 1
                    if button == "down": self.selected += 1

        self.selected %= self.total
        self.update_scroll(self.MAX_VISIBLE)
        self.draw_menu_waiting()

    def handle_moving(self, button):
        items = ["Stop"] + self.menu_items
        total = len(items)

        if button:
            if button != self.last_button:
                self.repeat_counter = 0

                if button == "up": self.selected -= 1
                if button == "down": self.selected += 1
                if button == "center":
                    if self.selected == 0:
                        self.command = "stop"
                        print("stop clicked")
                    else:
                        dest = items[self.selected]
                        self.command = dest
                        print("disp : New dest while moving:", dest)
                        self.selected = 0
                    return

            else:
                self.repeat_counter += 1
                if self.repeat_counter > 4:
                    self.repeat_counter %= 4
                    if button == "up": self.selected -= 1
                    if button == "down": self.selected += 1

        self.selected %= total
        self.update_scroll(self.MAX_VISIBLE)
        self.draw_menu_moving()

    def run(self):

        button = self.get_button()

        if self.mode == DISP_WAIT:
            self.handle_waiting(button)
        else:
            self.handle_moving(button)

        self.last_button = button
        time.sleep(0.05)

    def get_command(self):
        if self.command is None:
            return None
        cmd = self.command
        self.command = None
        return cmd
