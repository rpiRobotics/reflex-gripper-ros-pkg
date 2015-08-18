import curses

def center_justify(win, string):
    win_dim = win.getmaxyx() 
    current_cursor_pos = win.getyx()

    win_center_x = win_dim[1] / 2
    str_center_x = len(string) / 2
    
    new_cursor_pos_x = win_dim[1] - win_center_x -str_center_x
    
    win.addstr(current_cursor_pos[0], new_cursor_pos_x, string)
    win.refresh()

def curses_fingers_interface(screen, hand):
    # don't change the terminal colors
    curses.use_default_colors()
    # don't display the cursor
    curses.curs_set(0)
    
    SCREEN_X_DIM = screen.getmaxyx()[1]
    SCREEN_Y_DIM = screen.getmaxyx()[0]
    screen.refresh()

    title_win = curses.newwin(1,SCREEN_X_DIM, 0,0)
    title_str = "Interactive ReflexSF Controller"
    center_justify(title_win, title_str)
    
    keys_win = curses.newwin(13,50,1,0)
    instr="""\
                    tighten
                      _____
                     |     |
                     |  ^  |
                _____|_____|_____
               |     |     |     |
  prev finger  |  <  |  v  |  >  |  next finger
               |_____|_____|_____|
  
                     loosen
  q : Quit
"""
    keys_win.addstr(1,1,instr);
    keys_win.box()
    keys_win.refresh();

    finger_win = curses.newwin(1,SCREEN_X_DIM,14,0)
    fingers = hand.fingerNames
    finger_node_names = ["%s_%s"%(hand.namespace,finger) for finger in fingers]
    finger_index = 0;

    commands_win = curses.newwin(SCREEN_Y_DIM-15,SCREEN_X_DIM,15,0)
    commands_win.scrollok(True)
    commands_win.idlok(1)
    #commands_win.setscrreg(16,19)

    running = True
    while running:
        finger_win.erase()
        finger_win.addstr("Current Finger", curses.A_UNDERLINE)
        finger_win.addstr(": ")
        finger_win.addstr(fingers[finger_index % 4])
        finger_win.refresh()

        ch = screen.getkey()
        if ch == 'KEY_UP':
            commands_win.insertln()
            commands_win.addstr("Tightening %s\n"%(fingers[finger_index%4]))
            commands_win.move(0,0)
            #Tighten the finger
            hand.motors[finger_node_names[finger_index%4]].tighten()
        elif ch == 'KEY_DOWN':
            commands_win.insertln()
            commands_win.addstr("Loosening %s\n"%(fingers[finger_index%4]))
            commands_win.move(0,0)
            #Loosen the finger
            hand.motors[finger_node_names[finger_index%4]].loosen()
        elif ch == 'KEY_LEFT':
            finger_index -= 1
        elif ch == 'KEY_RIGHT':
            finger_index += 1
        elif ch == 'q':
            running = False
        commands_win.refresh()