defs = [
    "#define P?_INPUT() (DDR$ &= ~(1 << DD?))",
    "#define P?_OUTPUT() (DDR$ |= (1 << DD?))",
    "#define P?_PULLUP() (PORT$ |= (1 << P?))",
    "#define P?_PULLUP_OFF() (PORT$ &= ~(1 << P?))",
    "#define P?_ON() P?_PULLUP()",
    "#define P?_OFF() P?_PULLUP_OFF()",
    "#define P?_STATE() (PIN$ & (1 << PIN?))",
]

def generate(port, port_pin):
    for define in defs:
        print(define.replace("?", port_pin).replace("$", port))

generate("D", "D4")
