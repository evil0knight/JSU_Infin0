 # JSU_Infineon
	江苏大学英飞凌祖传代码============================24届杨奥
	学弟学妹们，见字如面，我将从0开始学习英飞凌，希望我的经历可以帮助你们，这篇.md将会以日记的方式呈现，（本人已经学完51单片机）。  
 	主要资料在B站和逐飞公众号里哦  
 # 日志
 ## **2025.1.16**:  
 这是我第三天打开英飞凌软件，还是不会用，结果一早上王zy学长突然来实验室了，教了我快1小时，that's amazing！，祝王zy学长考研顺利。**根据官方文档进入软件，重复操作set actie会导致example呈现灰色。ctrl+z是返回上一步**  
 **这个软件好像改版了，官方文档上下载按钮找不到，无法重复下载，目前办法是一直按着reset按钮，然后多试几次debug就行了（1.20发现解决办法了，右上角的小虫子，鼠标悬停名字叫Debug（不是旁边的winIDEA debug）,找到菜单里的红色方块，点掉）**  
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
  这里个你们隆重介绍一下我家的猫他叫[弟](https://github.com/evil0knight/JSU_Infin0/blob/main/images/5B9B17CED5BF7DACF951AFD515C9DF47.jpg)[弟](https://github.com/evil0knight/JSU_Infin0/blob/main/images/BB20DE6B64B784C16F2C725232924278.jpg)。   
  我有罪，今天没学多少，明天7点就起床，这小车我一定会搓出来的，明天一定好好学习。  
  --至车：  
  车，你是我无欲无求的人生里旁生出的一丝贪念。  
  车，钱的事你不用担心，我会说服我的队友，一起A钱买最好的配件，我想比赛那天，你一定美的很惊艳。  
  车，我更喜爱你的灵魂，我想你心里留下最美好的程序，别无所求，只希望你能每天蹦蹦跳跳的走在赛道上，让周围的一切都沾染上你的光，从此阳光灿烂，万物生长。--  
  #  **2025.1.20**:  
  车全到了等我把资料打包好全部上传。  
  今天学PID学一天，挺多的，方法就是在各种地方搜，太多了，我也很难总结。  
  9点了，晚上再学一个卡尔曼滤波，明天开始试用舵机，电机，看看能不能组装。  
  文件太多了，我分享了两个最重要的文件[逐飞轮腿准备流程](https://github.com/evil0knight/JSU_Infin0/blob/main/%E9%80%90%E9%A3%9E%E7%A7%91%E6%8A%80-20%E5%B1%8A%E5%B9%B3%E8%A1%A1%E8%BD%AE%E8%85%BF%E7%BB%84%E7%9B%B8%E5%85%B3%E8%B5%84%E6%96%99.xlsx),[轮腿组相关内容](https://github.com/evil0knight/JSU_Infin0/blob/main/%E9%80%90%E9%A3%9E%E7%A7%91%E6%8A%80-20%E5%B1%8A%E5%B9%B3%E8%A1%A1%E8%BD%AE%E8%85%BF%E7%BB%84%E7%9B%B8%E5%85%B3%E8%B5%84%E6%96%99.xlsx)，里面有各种文章和网盘，不知道网盘密码会不会改，模块代码买了学习套件就会给，不急这一会儿  
  不懂得地方直接搜，B站，百度，AI。  
  #  **2025.1.21**：  
  **逐飞助手打不开只要请你学长吃一顿肯德基再把中文名改成英文名就行了**   
  理发师你说这个发型好看的时候良心不痛吗，没法过年了🥲。  
  在写注释，明天看看能不能焊小车，明天把写好的注释提交。  
  已经有人把小车站起来了，我5天之内争取  
  #  **2025.1.22**：  
  **小车需要焊接，焊接只要差不多拔不下来就行，焊多了可以趁融化的时候把他甩下来，小经验**  
  拧螺丝的时候可以先拿一个小螺丝攻丝，然后在上M3*5+6.  
  **看安装驱动板安装版的驱动板方向再焊接驱动板，都是一一对应的😭**  
  今天花了一天时间看了学长写的函数，我滴妈，太复杂了，估计还要看两天才能看懂怎么用。我把代码都用GPT写了注释，gpt比KIMI好用点，吧，好用一点，用freecat翻墙，网上应该有翻墙教程吧，我这还是rm学长教我的，可惜叛变到智能车了，sorry.    
  不过这车是真帅，真TM的帅，无敌帅，帅无敌。  
  靓车爆照[1图](https://github.com/evil0knight/JSU_Infin0/blob/main/images/1.jpg),[2图](https://github.com/evil0knight/JSU_Infin0/blob/main/images/2.jpg),[3图](https://github.com/evil0knight/JSU_Infin0/blob/main/images/3.jpg),[4图](https://github.com/evil0knight/JSU_Infin0/blob/main/images/4.jpg).  
  #  **2025.1.23**:  
  **电池到了一定要先充电，一定要买配套电池，那个官方的驱动历程，没有电池电机是转不起来的，蓝灯，提示音也不会响，电压问题**  
  **官方文档TC264_Library-master-->【文档】ADS使用教程 芯片手册等，这个挺重要，看看例程的功能**  
  今天实验了电机，陀螺仪，都能正常工作，还好没有把驱动焊坏。  
  陀螺仪的加速度怎么静止都上千了，为啥  
  官方发的历程都要好好看，估计最后就是这些历程拼图造小车  
  这几天手机又看多了，红果短剧控人有一手啊，不过我只看广告。  
  我要早起学习，群里佬都完赛了，呜呜呜呜  
  #  **2025.1.24**:  
  sleep  
 **磁编不是编码器历程**，**在驱动里有获取电机速度，助手里也有电机调试界面！！！**，我一上午没了😭😭😭😭😭😭😭😭😭😭😭😭😭😭，在群里问也没人鸟我，我好笨啊。完了，群好安静  
 弟弟在外面玩了一上午，喜欢吃树叶的猫[小猫呲牙](images/小猫呲牙.jpg),[小猫吐舌头](images/小猫吐舌头.jpg),家里的🐟躺着睡，但是还没死，估计喂多了气囊移位了  
 **驱动例程里是通过中断来获取磁编信息的，在isc.c的234行，我是通过左键点search text->file就可以找到这句话都在哪里出现了，通过call back(大概名字)if里面还有个读取函数，然后就可以读取信息了**  
 这又是谁的一下午😭😭😭😭😭😭😭。找到234行真的是直接站起来了。  
 磁编文件menc大概是用不了的，我才是因为驱动板程序都是写好的，只能传速度，但也够了，现在硬件都正常使用。**逐飞助手left speed,right speed数据为零可以按一下reset,这个又花了我好长时间😭😭**明天还是不打算开始写PID，去网上搜开源的类似的轮腿机器人，双轮平衡小车，这些代码的整体框架还是要学别人的，学长的是独轮组，而且代码很乱很丑。  
 
  
  
