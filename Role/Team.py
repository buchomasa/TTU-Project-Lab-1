from machine import Pin

teamColor = Pin(0, Pin.IN, Pin.PULL_UP)
def pickTeam():
    purpleTeam = False
    yellowTeam = False
    if teamColor.value() == 0:
        purpleTeam = True
        yellowTeam = False
        print("Purple team selected")
    
    else:
        purpleTeam = False
        yellowTeam = True
        print("Yellow team selected")
    
pickTeam()
