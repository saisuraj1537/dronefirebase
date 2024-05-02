import pyautogui
import time
pyautogui.FAILSAFE = False
time.sleep(5)
pyautogui.moveTo(0,0)
for i in range(143):
    pyautogui.write('hii')
    pyautogui.press('enter')