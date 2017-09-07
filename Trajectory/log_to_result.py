


rfile = open('log.txt')
wfile = open('result.txt', 'w')
result = []
result2 = ""

for line in rfile:
    result = line.split(" ")
    x = 0
    
    for i in result:
        if(x%5==0):
            result2 = result2 + i + " "
        if(x%5==1):
            result2 = result2 + i + " "
        if(x%5==2):
            result2 = result2 + i + "\n"
        x += 1
    wfile.write(result2)
    result2 = ""
            
