for j in range(1,26):
    f = open("warehouse-10-20-10-2-2/scen-even/warehouse-10-20-10-2-2-even-"+str(j)+".scen")
    lines = f.read()
    #print (lines)
    #print(type(lines))

    line_list = lines.split('\n')[1:-1]

    f.close()

    my_list = []

    high = line_list[0].split('\t')[2]
    width = line_list[0].split('\t')[3]
    starts = []
    tasks = []

    for i in range(len(line_list)):
        temp = line_list[i].split('\t')[4:8]
        start = int(temp[0])*int(width) + int(temp[1])
        task = int(temp[2])*int(width) + int(temp[3])
        starts.append(str(start))
        tasks.append(str(task))

    # print(starts)
    # print(tasks)


    fo = open("warehouse-10-20-10-2-2/mapf_competition/even/agents-"+str(j)+".txt", "w")
    fo.write(str(len(starts)) + "\n" + "\n".join(starts))
    
    
    fo.close()

    fo = open("warehouse-10-20-10-2-2/mapf_competition/even/tasks-"+str(j)+".txt", "w")
    fo.write(str(len(tasks)) + "\n" + "\n".join(tasks))
    
    
    fo.close()