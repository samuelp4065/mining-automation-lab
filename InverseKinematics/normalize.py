# Boom
def angleToEncoder1(angle):
    return int((angle - 40) * 9/2)

# Stick
def angleToEncoder2(angle):
    return int((angle + 79) * (-90/41))

# Bucket
def angleToEncoder3(angle):
    return int((angle + 385/3) * (-27/25))


while True:
    command = input("Enter angles (th1 th2 th3) in degrees, separated by spaces (or 'exit' to quit): ")

    if command.lower() == 'exit':
        break

    parts = command.split()
    if len(parts) != 3:
        print("❌ Please enter exactly 3 angles (th1 th2 th3).")
        continue

    try:
        th1, th2, th3 = map(float, parts)
    except ValueError:
        print("❌ Invalid input. Please enter numeric values.")
        continue

    enc1 = angleToEncoder1(th1)
    enc2 = angleToEncoder2(th2)
    enc3 = angleToEncoder3(th3)

    print(f"Encoder values:")
    print(f"  Boom   → {enc1}")
    print(f"  Stick  → {enc2}")
    print(f"  Bucket → {enc3}\n")
