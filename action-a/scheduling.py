ENTRYPOINT ["/entrypoint.sh"]
#sum1=0
from main_algorithm import NSGA-II
k=1000
for i in range(1):
    import datetime
    starttime = datetime.datetime.now()
    model=NSGA_II(200,100,200)
    model.main()
    endtime = datetime.datetime.now()
    print(model.min_sum)
    print (endtime - starttime)
    if k>int(model.min_sum):
        k=int(model.min_sum)
        l=model.t_mac
    
k
