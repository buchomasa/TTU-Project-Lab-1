from machine import Pin

roleSelect = Pin(8, Pin.IN, Pin.PULL_UP)

def pickRole():
    if roleSelect.value() == 0:
        print("Striker")
        return "striker"
    else:
        print("Goalie")
        return "goalie"


role = pickRole()
print("Selected role:", role)
