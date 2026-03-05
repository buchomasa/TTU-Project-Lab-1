from machine import Pin

roleSelect = Pin(3, Pin.IN, Pin.PULL_UP)
def pickRole():
    striker = False
    goalie = False
    if roleSelect.value() == 0:
        striker = True
        goalie = False
        print("Striker")
        
    else:
        striker = False
        goalie = True
        print("Goalie")
        
pickRole()
