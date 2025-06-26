# Example file showing a circle moving on screen
import pygame

class GuiHandler():

    def __init__(
        self, screen_height: int=720, screen_width: int=1280,
        text_color: str="white", bg_color: str="black"):
        self.__screen_height = screen_height
        self.__screen_width = screen_width
        self.__text_color = pygame.Color(text_color)
        self.__bg_color = pygame.Color(bg_color)
        pygame.init()  # init pygame
        pygame.key.set_repeat(420, 42)  # enable repeating key input, if key is constantly pressed
        self.__screen = pygame.display.set_mode((self.__screen_width, self.__screen_height))
        self.__std_text_font = pygame.font.Font(None, 42)
        self.__clock = pygame.time.Clock()
        self.__to_blit: list = []
        self.terminal_content: list = []
        self.__command_history_file_path = "printer_control/terminal_history"
        self.__command_history: list = self.load_command_history()
        self.__information_content_left: dict = {}
        self.__information_content_center: dict = {}
        self.__information_content_right: dict = {}
        self.__running_input: bool = True
        # define window part heights:
        title_height = self.__screen_height//10
        in_rect_height = self.__screen_height//20
        stats_rect_height = self.__screen_height//7
        out_rect_height = self.__screen_height-in_rect_height-stats_rect_height-title_height
        # define window part positions (by defining top left corner):
        self.__out_rect_top = self.__screen_height//10
        self.__in_rect_top = self.__screen_height - stats_rect_height - in_rect_height
        stats_rect_top = self.__screen_height - stats_rect_height
        # initialize the window parts as pygame rects:
        self.__title_rect = pygame.Rect(0, 0, self.__screen_width, title_height)
        self.__out_rect = pygame.Rect(0, self.__out_rect_top, self.__screen_width, out_rect_height)
        self.__in_rect = pygame.Rect(0, self.__in_rect_top, self.__screen_width, in_rect_height)
        self.__stats_rect_left = pygame.Rect(0, stats_rect_top, self.__screen_width//3, stats_rect_height)
        self.__stats_rect_center = pygame.Rect(self.__screen_width//3, stats_rect_top, self.__screen_width//3, stats_rect_height)
        self.__stats_rect_right = pygame.Rect(2*self.__screen_width//3, stats_rect_top, self.__screen_width//3, stats_rect_height)

    def load_command_history(self) -> list:
        with open(self.__command_history_file_path, "r", encoding="utf-8") as reader:
            lines = reader.readlines()
            command_history: list = []
            for line in lines:
                line = line.strip("\n")
                if line:
                    command_history.append(line)
        if not command_history:
            command_history = [""]
        return command_history

    def write_command_history(self) -> None:
        with open(self.__command_history_file_path, "w", encoding="utf-8") as writer:
            for command in self.__command_history:
                writer.write(f"{command}\n")

    def startup(self):
        font = pygame.font.Font(None, 150)
        running = True
        time = 0
        print_str = "S t a r t i n g . . ."
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            sliced_str = print_str[:int(time)]
            if int(time) == len(print_str) + 1:
                running = False
            self.__screen.fill("black")
            text_surface = font.render(sliced_str, True, pygame.Color("green"))
            self.__screen.blit(text_surface, (60, self.__screen_height//2))
            pygame.display.flip()
            time += self.__clock.tick(60) / 420

    def refresh(self):
        # make title
        title_font = pygame.font.Font(None, 90)
        title = "2D_Printer_control"
        title_surface = title_font.render(title, True, self.__text_color)
        title_rect = title_surface.get_rect(center=(self.__screen_width//2, 50))
        font = self.__std_text_font
        # flip() the display to put your work on screenterminal
        self.__screen.fill(self.__bg_color)  # clear screen
        #self.debug_rects()  # color the screen sections
        self.__screen.blit(self.__input, (10, self.__in_rect_top))  # blit user input
        self.__screen.blits(self.__to_blit)  # blit the terminal_content to the screen
        self.__screen.blit(title_surface, title_rect)  # blit Title to screen
        # render and blit the information content:
        if self.__information_content_left:
            info_content_left = str(self.__information_content_left)
            info_content_left_surface = font.render(info_content_left, True, self.__text_color)
            self.__screen.blit(info_content_left_surface, self.__stats_rect_left)
        if self.__information_content_center:
            info_content_center = str(self.__information_content_center)
            info_content_center_surface = font.render(info_content_center, True, self.__text_color)
            self.__screen.blit(info_content_center_surface, self.__stats_rect_center)
        if self.__information_content_right:
            info_content_right = str(self.__information_content_right)
            info_content_right_surface = font.render(info_content_right, True, self.__text_color)
            self.__screen.blit(info_content_right_surface, self.__stats_rect_right)
        pygame.display.flip()
        # limits FPS to 60
        self.__clock.tick(60)

    def debug_rects(self):
        """prints the different sections
        of the window in bright colors"""
        # define colors:
        bg_color = pygame.Color("black")
        fg_color = pygame.Color("green")
        out_color = pygame.Color("yellow")
        in_color = pygame.Color("red")
        stats_color_left = pygame.Color("blue")
        stats_color_center = pygame.Color("aqua")
        stats_color_right = pygame.Color("cornflowerblue")
        # draw rects:
        pygame.draw.rect(self.__screen, fg_color, self.__title_rect)
        pygame.draw.rect(self.__screen, in_color, self.__in_rect)
        pygame.draw.rect(self.__screen, stats_color_left, self.__stats_rect_left)
        pygame.draw.rect(self.__screen, stats_color_center, self.__stats_rect_center)
        pygame.draw.rect(self.__screen, stats_color_right, self.__stats_rect_right)
        #pygame.draw.rect(self.__screen, out_color, self.__out_rect)

    def new_print(self, text):
        text = str(text)
        self.terminal_content.append(text)
        line_height = pygame.font.Font.size(self.__std_text_font, "Tqpg")[1]
        upper_bound = self.__out_rect_top
        lower_bound = self.__in_rect_top
        while len(self.terminal_content)*line_height > lower_bound -upper_bound:  # check if content fits in out_rect
            self.terminal_content.pop(0)  # pop oldest content
        # render text to a list of text_surfaces:
        text_surfaces = [self.__std_text_font.render(i, True, self.__text_color) for i in self.terminal_content]
        self.__to_blit = []  # reset self.__to_blit
        for index, text_surface in enumerate(reversed(text_surfaces)):
            self.__to_blit.append((text_surface, (10, lower_bound - (index+1) * line_height)))

    def clear(self):
        self.terminal_content = []
        self.__to_blit = []

    def stop_input(self):
        self.__running_input = False

    def new_input(self, prompt = "input>"):
        prompt = f"{prompt.strip(' ')} " # make sure promt contains exactly one space at the end
        font = self.__std_text_font
        self.__running_input = True
        user_input: str = ""
        user_input_tmp_history: str = None
        command_history_index = -1
        while self.__running_input:
            # pygame.QUIT event means the user clicked X to close your window
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.__running_input = False
                    self.write_command_history()
                    return "q" # exit the programm
                # text input:
                elif event.type == pygame.TEXTINPUT:
                    user_input += event.text
                # special keys input:
                elif event.type == pygame.KEYDOWN:
                    # arrow key up:
                    if event.key == pygame.K_UP:
                        if not user_input_tmp_history:
                            user_input_tmp_history = user_input
                        command_history_index +=1
                        if command_history_index == len(self.__command_history):
                            command_history_index -=1
                        if command_history_index == -1:
                            user_input = user_input_tmp_history
                        else:
                            user_input = self.__command_history[command_history_index]
                    # arrow key down:
                    elif event.key == pygame.K_DOWN:
                        if not user_input_tmp_history:
                            user_input_tmp_history = user_input
                        command_history_index -=1
                        if command_history_index <= -2:
                            user_input = ""
                            command_history_index = -2
                        elif command_history_index == -1:
                            user_input = user_input_tmp_history
                        else:
                            user_input = self.__command_history[command_history_index]
                    # enter key input:
                    elif event.key == pygame.K_RETURN:
                        if user_input.strip(" ") != "" and user_input.strip(" ") != self.__command_history[0]:
                            self.__command_history.insert(0, user_input)
                        command_history_index = -1
                        user_input_tmp_history = None
                        if user_input.lower() == "clear":
                            self.clear()
                            user_input = ""
                        elif user_input:
                            return user_input
                    elif event.key == pygame.K_BACKSPACE:
                        user_input = user_input[:-1]
            self.__input = font.render(f"{prompt}{user_input}", True, self.__text_color)
            # blit user input to screen:
            self.refresh()

    def set_information_left(self, key, value):
        self.__information_content_left[key] = value

    def set_information_center(self, key, value):
        self.__information_content_center[key] = value

    def set_information_right(self, key, value):
        self.__information_content_right[key] = value

    def reset_information(self):
        self.__information_content_left = {}
        self.__information_content_center = {}
        self.__information_content_right = {}


if __name__ == "__main__":
    gui_handler = GuiHandler()
    #gui_handler.startup()
    while True:
        text = gui_handler.new_input()
        if text.lower() == "exit":
            break
        else:
            gui_handler.new_print(text)
            gui_handler.refresh()
    pygame.quit()