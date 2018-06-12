for x in xrange(1,10):
    filename = 'watchtower0'+str(x)+'.robot.yaml'
    file = open(filename, 'w+')
    file.write("owner: eth_idsc \nhostname: watchtower0"+str(x) +"\nusername: mom\ndescription: watchtower, RaspberryPi with Camera\nlog:")
    file.close()
for x in xrange(10,51):
    filename = 'watchtower'+str(x)+'.robot.yaml'
    file = open(filename, 'w+')
    file.write("owner: eth_idsc \nhostname: watchtower"+str(x) +"\nusername: mom\ndescription: watchtower, RaspberryPi with Camera\nlog:")
    file.close()
