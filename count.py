import os

num = []
with open("./build/city_path_man.txt") as f:
    for n in f.readlines():
        try:
            num.append(int(n))
        except ValueError:
            pass
print(sum(num)/len(num))
