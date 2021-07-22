## Introduction 

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

**The directory structure of simulator (模拟器的目录结构)**

主函数--main_algorithm.py
  
  案例可以自己调整，文件位置需要修改。
  
  修改案例后，需要将机器和工件参数进行修改：
      
      def data_pro(self):
        num_mac = 15   #机器总数量
        self.num_mac=num_mac
        self.num_p = 20 #工件总数

调用函数--scheduling.py
