 # JSU_Infineon
	江苏大学英飞凌祖传代码============================24届杨奥
	学弟学妹们，见字如面，我将从0开始学习英飞凌，希望我的经历可以帮助你们，这篇.md将会以日记的方式呈现，（本人已经学完51单片机）。  
 	主要资料在B站和逐飞公众号里哦  
 # 日志
 ## **2025.1.16**:  
 这是我第三天打开英飞凌软件，还是不会用，结果一早上王zy学长突然来实验室了，教了我快1小时，that's amazing！，祝王zy学长考研顺利。**根据官方文档进入软件，重复操作set actie会导致example呈现灰色。ctrl+z是返回上一步**  
 **这个软件好像改版了，官方文档上下载按钮找不到，无法重复下载，目前办法是一直按着reset按钮，然后多试几次debug就行了**
 **重要图片，详情看学长给的小车建造思路**  
 [WZY学长笔记](https://github.com/evil0knight/JSU_Infin0/blob/main/images/wzy%E5%AD%A6%E9%95%BF%E7%9A%84%E5%B0%8F%E8%BD%A6%E6%9E%84%E5%BB%BA%E9%87%8D%E8%A6%81%E6%80%9D%E8%B7%AF.jpg)
 上图学长笔记，可是我还没学编码器，陀螺仪。  

 ## **2025.1.17**： 
 终于期末考完了.  
 原谅我一下午什么也没干，我刚考完啊，睡醒5点了  
 我去吃饭   
 **车模买了，问客服要资料没有，车模不包含主板驱动板，除了车模还要有学习主板，驱动板我的建议是直接买学习套件1537元，可是这个学习套件里没有驱动板，**  
 **刚刚看了一下，学习板改了，还是重新买学习板吧，不要学长的了。（排针都歪了，资料也好乱，太难受了）明天9点问一下客服买学习板给不给资料，买驱动板给不给资料，买4路舵机电源模块给不给资料**   
 **总的就是告诉客服，我有核心板，车模，英飞凌下载器，那要一辆完整的车还差什么，管他要核心版，驱动板，电源舵机模块的引脚，以及各种资料。（1月21停止发货，在这之前必须搞定）**  
 **主板资料：[https://gitee.com/seekfree/TC264_Library](https://gitee.com/seekfree/TC264_Library/tree/master/Seekfree_TC264_Opensource_Library)，驱动板资料：[https://gitee.com/seekfree/CYT2BL3_Brushless_Driver_Project](https://gitee.com/seekfree/CYT2BL3_Brushless_Driver_Project)**  
 我的队友好高冷，要感冒了，😢😢，能不能理我一下。抓狂！！！  
  ## **2025.1.18**： 
  牛魔的，轮腿组小群连发21条消息，21小时过去，0个人回复，失踪了？想报警。  
  回复了，but it's too late to apologize,it's too late(🎶);  
  **微信公众号逐飞科技，车模浅析,轮腿组准备流程**  
  上午看whx玩游戏，5把赢了1把，乐，农p的丑态。  
  **学习套件买了，冲，如果像我一样，有核心板和英飞凌下载器，直接跟客服说，他会帮你改价，直接下单学习套件，提交订单，等他改价，再付款，到货之后如果不退货，他才会发学习资料，许多学习资料是要等车模来了之后才能有的。**   
  为毛我还在学校，成留守儿童了，悲  
  终于到家了，爽。现在就差学习套件的资料了，明天把最近搜到的信息在看一下，再在B站上找点ADS,智能车，卡尔曼滤波，之类的视频。  
  ##  **2025.1.19**：
  **对比度没搜到，图像采集出来的是一个矩阵，按照总钻风采集出来的图像，矩阵上每个点由白（0）到黑（255）的一个色（0~255），就是个二维数组，二值化的意思是先用大津法（一种图像算法，好像照抄就好了，不确定）算出一个阈值，小于这个阈值就是白（0），大于这个就是黑（255），这样算法会方便易懂，逆势化就是把图像变成一个上帝视角，采集到的路都是横平竖直的，这些是从B站上看的，UP：WwuSama的智能车入门基础教学合集里看的。目前来看，这个合集很重要，八邻域我没搜，根据WZY学长所述，就是有时候图像会采集到白块，如果这个白块4周都是黑色，那他就是黑色，黑色同理**  
  **舵机的原理：他是一个伺服系统，意思是输出可以影响输入，转动的动力来源只是一个普通电机，通过齿轮组加大扭力，当到达设定角度时，电机被断电，设定角度的测量通过一个芯片测量，输入信息时PWM，高电压时间越长，就越顺时针转（大概这个意思），舵机旋转角度不要太大，舵机容易坏，装在车上的时候，舵机角度有限，一定要会了在装**  
  车模到了，吃完饭就可以迎接我的爱车了.  
  车模3部分[车模](https://github.com/evil0knight/JSU_Infin0/blob/main/images/9311554847967D365C688860AB1F6429.jpg),[收据](https://github.com/evil0knight/JSU_Infin0/blob/main/images/98A2E7EA33E9AA6A04E9D717F7FDEE33.jpg),[说明书](https://github.com/evil0knight/JSU_Infin0/blob/main/images/67C884BD3F96ADC681F1E695504C7463.jpg);
  这里个你们隆重介绍一下我家的猫他叫[弟](https://github.com/evil0knight/JSU_Infin0/blob/main/images/5B9B17CED5BF7DACF951AFD515C9DF47.jpg)[弟](https://github.com/evil0knight/JSU_Infin0/blob/main/images/BB20DE6B64B784C16F2C725232924278.jpg)  
  
  
