 # JSU_Infineon
	江苏大学英飞凌祖传代码============================24届杨奥
	学弟学妹们，见字如面，我将从0开始学习英飞凌，希望我的经历可以帮助你们，这篇.md将会以日记的方式呈现，（本人已经学完51单片机）。  
 	主要资料在B站和逐飞公众号里哦  
 # 日志
 ## **2025.1.16**:  
 这是我第三天打开英飞凌软件，还是不会用，结果一早上王zy学长突然来实验室了，教了我快1小时，that's amazing！，祝王zy学长考研顺利。**根据官方文档进入软件，重复操作set actie会导致example呈现灰色。ctrl+z是返回上一步**  
 **这个软件好像改版了，官方文档上下载按钮找不到，无法重复下载，目前办法是一直按着reset按钮，然后多试几次debug就行了（1.20发现解决办法了，右上角的小虫子，鼠标悬停名字叫Debug（不是旁边的winIDEA debug）,找到菜单里的红色方块，点掉）**  
 ~~**重要图片，详情看学长给的小车建造思路**~~ **（2.9来pid别看）**  
 [WZY学长笔记](https://github.com/evil0knight/JSU_Infin0/blob/main/images/wzy%E5%AD%A6%E9%95%BF%E7%9A%84%E5%B0%8F%E8%BD%A6%E6%9E%84%E5%BB%BA%E9%87%8D%E8%A6%81%E6%80%9D%E8%B7%AF.jpg)
 上图学长笔记，可是我还没学编码器，陀螺仪。  

 ## **2025.1.17**： 
 终于期末考完了.    
 **车模买了，问客服要资料没有，车模不包含主板驱动板，除了车模还要有学习主板，驱动板我的建议是直接买学习套件1537元，可是这个学习套件里没有驱动板，**  
 **刚刚看了一下，学习板改了，还是重新买学习板吧，不要学长的了。（排针都歪了，资料也好乱，太难受了，后期：那个主板是独轮的主板，不一样，要自己买轮腿的）明天9点问一下客服买学习板给不给资料，买驱动板给不给资料，买4路舵机电源模块给不给资料**   
 **总的就是告诉客服，我有核心板，车模，英飞凌下载器，那要一辆完整的车还差什么，管他要核心版，驱动板，电源舵机模块的引脚，以及各种资料。（1月21停止发货，在这之前必须搞定）**  
 **主板资料：[https://gitee.com/seekfree/TC264_Library](https://gitee.com/seekfree/TC264_Library/tree/master/Seekfree_TC264_Opensource_Library)，驱动板资料：[https://gitee.com/seekfree/CYT2BL3_Brushless_Driver_Project](https://gitee.com/seekfree/CYT2BL3_Brushless_Driver_Project)**    
  ## **2025.1.18**： 
  **微信公众号逐飞科技，车模浅析,轮腿组准备流程**  
  上午看whx玩游戏，5把赢了1把，乐，农p的丑态。  
  **学习套件买了，冲，如果像我一样，有核心板和英飞凌下载器，直接跟客服说，他会帮你改价，直接下单学习套件，提交订单，等他改价，再付款，到货之后如果不退货，他才会发学习资料，许多学习资料是要等车模来了之后才能有的。**   
  终于到家了，爽。现在就差学习套件的资料了，明天把最近搜到的信息在看一下，再在B站上找点ADS,智能车，卡尔曼滤波，之类的视频。  
  ##  **2025.1.19**：
  **对比度没搜到，图像采集出来的是一个矩阵，按照总钻风采集出来的图像，矩阵上每个点由白（0）到黑（255）的一个色（0~255），就是个二维数组，二值化的意思是先用大津法（一种图像算法，好像照抄就好了，不确定）算出一个阈值，小于这个阈值就是白（0），大于这个就是黑（255），这样算法会方便易懂，逆势化就是把图像变成一个上帝视角，采集到的路都是横平竖直的，这些是从B站上看的，UP：WwuSama的智能车入门基础教学合集里看的。目前来看，这个合集很重要，八邻域我没搜，根据WZY学长所述，就是有时候图像会采集到白块，如果这个白块4周都是黑色，那他就是黑色，黑色同理**  
  **舵机的原理：他是一个伺服系统，意思是输出可以影响输入，转动的动力来源只是一个普通电机，通过齿轮组加大扭力，当到达设定角度时，电机被断电，设定角度的测量通过一个芯片测量，输入信息时PWM，高电压时间越长，就越顺时针转（大概这个意思），舵机旋转角度不要太大，舵机容易坏，装在车上的时候，舵机角度有限**  
  车模到了，吃完饭就可以迎接我的爱车了.  
  车模3部分[车模](https://github.com/evil0knight/JSU_Infin0/blob/main/images/9311554847967D365C688860AB1F6429.jpg),[收据](https://github.com/evil0knight/JSU_Infin0/blob/main/images/98A2E7EA33E9AA6A04E9D717F7FDEE33.jpg),[说明书](https://github.com/evil0knight/JSU_Infin0/blob/main/images/67C884BD3F96ADC681F1E695504C7463.jpg);   
  这里个你们隆重介绍一下我家的猫他叫[弟](https://github.com/evil0knight/JSU_Infin0/blob/main/images/5B9B17CED5BF7DACF951AFD515C9DF47.jpg)[弟](https://github.com/evil0knight/JSU_Infin0/blob/main/images/BB20DE6B64B784C16F2C725232924278.jpg)。   
  ~~至车：  
  车，你是我无欲无求的人生里旁生出的一丝贪念。  
  车，钱的事你不用担心，我会说服我的队友，一起A钱买最好的配件，我想比赛那天，你一定美的很惊艳。  
  车，我更喜爱你的灵魂，我想你心里留下最美好的程序，别无所求，只希望你能每天蹦蹦跳跳的走在赛道上，让周围的一切都沾染上你的光，从此阳光灿烂，万物生长。~~  
  ##  **2025.1.20**:  
  车全到了等我把资料打包好全部上传。  
  9点了，晚上再学一个卡尔曼滤波，明天开始试用舵机，电机，看看能不能组装。  
  文件太多了，我分享了两个最重要的文件[逐飞轮腿准备流程](https://github.com/evil0knight/JSU_Infin0/blob/main/%E9%80%90%E9%A3%9E%E7%A7%91%E6%8A%80-20%E5%B1%8A%E5%B9%B3%E8%A1%A1%E8%BD%AE%E8%85%BF%E7%BB%84%E7%9B%B8%E5%85%B3%E8%B5%84%E6%96%99.xlsx),[轮腿组相关内容](https://github.com/evil0knight/JSU_Infin0/blob/main/%E9%80%90%E9%A3%9E%E7%A7%91%E6%8A%80-20%E5%B1%8A%E5%B9%B3%E8%A1%A1%E8%BD%AE%E8%85%BF%E7%BB%84%E7%9B%B8%E5%85%B3%E8%B5%84%E6%96%99.xlsx)，里面有各种文章和网盘，不知道网盘密码会不会改，模块代码买了学习套件就会给，不急这一会儿  
  不懂得地方直接搜，B站，百度，AI。  
  ##  **2025.1.21**：  
  **逐飞助手打不开只要请你学长吃一顿肯德基再把中文名改成英文名就行了**   
  理发师你说这个发型好看的时候良心不痛吗，没法过年了🥲。  
  在写注释，明天看看能不能焊小车，明天把写好的注释提交。  
  已经有人把小车站起来了，我5天之内争取  
  ##  **2025.1.22**：  
  **小车需要焊接，焊接只要差不多拔不下来就行，焊多了可以趁融化的时候把他甩下来，小经验**  
  拧螺丝的时候可以先拿一个小螺丝攻丝，然后在上M3*5+6.  
  **看安装驱动板安装版的驱动板方向再焊接驱动板，都是一一对应的😭**  
  今天花了一天时间看了学长写的函数，我滴妈，太复杂了，估计还要看两天才能看懂怎么用。我把代码都用GPT写了注释，gpt比KIMI好用点，吧，好用一点，用freecat翻墙，网上应该有翻墙教程吧，我这还是rm学长教我的，可惜叛变到智能车了，sorry.    
  不过这车是真帅，真TM的帅，无敌帅，帅无敌。  
  靓车爆照[1图](https://github.com/evil0knight/JSU_Infin0/blob/main/images/1.jpg),[2图](https://github.com/evil0knight/JSU_Infin0/blob/main/images/2.jpg),[3图](https://github.com/evil0knight/JSU_Infin0/blob/main/images/3.jpg),[4图](https://github.com/evil0knight/JSU_Infin0/blob/main/images/4.jpg).  
  ##  **2025.1.23**:  
  **电池到了一定要先充电，一定要买配套电池，那个官方的驱动历程，没有电池电机是转不起来的，蓝灯，提示音也不会响，电压问题**  
  **官方文档TC264_Library-master-->【文档】ADS使用教程 芯片手册等，这个挺重要，看看例程的功能**  
  今天实验了电机，陀螺仪，都能正常工作，还好没有把驱动焊坏。  
  陀螺仪的加速度怎么静止都上千了，为啥（1.27因为没滤波，噪音很大）  
  官方发的历程都要好好看，估计最后就是这些历程拼图造小车    
  我要早起学习，群里佬都完赛了，呜呜呜呜  
  ##  **2025.1.24**:  
  sleep  
 **磁编不是编码器历程**，**在驱动里有获取电机速度，助手里也有电机调试界面！！！**，我一上午没了😭😭😭😭😭😭😭😭😭😭😭😭😭😭，在群里问也没人鸟我。完了，群好安静  
 弟弟在外面玩了一上午，喜欢吃树叶的猫[小猫呲牙](images/小猫呲牙.jpg),[小猫吐舌头](images/小猫吐舌头.jpg),家里的🐟躺着睡，但是还没死，估计喂多了气囊移位了  
 **驱动例程里是通过中断来获取磁编信息的，在isc.c的234行，我是通过左键点search text->file就可以找到这句话都在哪里出现了，通过call back(大概名字)if里面还有个读取函数，然后就可以读取信息了**  
 这又是谁的一下午😭😭😭😭😭😭😭。找到234行真的是直接站起来了。  
 磁编文件menc大概是用不了的，我猜是因为驱动板程序都是写好的，只能传速度，但也够了，现在硬件都正常使用。**逐飞助手left speed,right speed数据为零可以按一下reset,这个又花了我好长时间😭😭**明天还是不打算开始写PID，去网上搜开源的类似的轮腿机器人，双轮平衡小车，这些代码的整体框架还是要学别人的，学长的是独轮组，而且代码很乱很丑。  
 🐟死了。  
  LQR模型要精确，建模我们轮腿太灵活，不能用~~我不会~~，这两天我要看完全网的PID程序。💀。  
  ##  **2025.1.25**  
  卡尔曼滤波，嘿嘿，曼波，曼波。。
  互补滤波可能没用，（the day after tomorrow代码里有互补滤波）还在试，PID是串级的，内环角度，外环角速度，再外环速度。  
  曼波，我还要拿胶带粘陀螺仪，什么鬼设计。  
  陀螺仪插回去插反了，烧了，完了  
  没坏，差点破费40.吓死了，不学了，大难不死   
  ##  **2025.1.26**  
  卡尔曼出来了！！！！！！！！！  
  **中断函数初始化一定要放在所有初始化最后，这个问题排查一上午，当遇到没输出的时候，用printf("1");一行一行试！**  
  今天是左巴扬逝世第5年纪念日，左巴扬是谁？左巴扬是一位美国飞行员，有超过1200hour的S-76直升飞机飞行经验。在5年前的今天，左巴扬驾驶直升飞机时发生坠机事件，直升机内包括机长左巴扬和球星科比等机组人员都不幸遇难  
  **"ekf.h""matrix.h"这两个头文件放进zf_common_headfile.h用户层，要用的函数初始化一下，再把EKF_UpData()放中断里就行了。yaw不准，会离散，不能用，但是pitch和roll是可以用的，目前由于陀螺仪放置问题，我pitch和roll是反的**  
  5天之约我输了，小车要等年后了  
  ##  **2025.1.27**  
  **pid有很多种比如位置式，增量式，抗饱和积分，变速积分，前馈微分，低通滤波微分，专家pid，模糊pid**  
  还有些用来装逼的pid比如什么神经网络pid，遗传pid，灰色pid，无聊的时候可以看看  
  **资料关注微信公众号“工科极客”，发送“PID”，即可领取pid深入资料**  
  下面仅个人意见：  
  **串级pid,注意模块化编程，两个轮子都要pid,站立可以从资料最多的位置式入手，以后再开始增量式，最内环是角速度5ms，中环是角度5ms，根据卡尔曼滤波出来的，卡尔曼我目前放主函数了，后面应该还会改，文件不用改，直接套就行了，最外环是速度环10ms，代码网上满天飞，看** 
  我的也好，不过我很菜，去网上吃点好的  
  可惜过年断更了，今天最多内环调完，2月2回来学  
  patience,is,the key in life  
  **内环pid调失败了，~~因为角速度太敏感了~~（来自未来：内环是角度）并且我按照网上的方法先调P，不能像网上那样一个换一个环调，我们要全写出来一起调，春节好好研究pid,加油**  
  ##  **2025.2.2**  
  内环PI就有平衡趋势了，p:1.4,i:0.05,积分限幅2000，输出限幅5000；目前他往前走，有时会抽搐。  
  ##  **2025.2.3**  
  **不小心关掉了一些栏目，Window → Perspective → Reset Perspective**  
  参数一点不会调，不管怎么改都是抽搐或者向前冲，中环输出为0也不知道什么毛病~~2025.2.9因为角速度不能放内环啊哥~~2025.2.11：感置疑学长，什么实力？  
  ##  **2025.2.4**  
  学不下去，感觉要死在家里变成一滩狗屎，想要一个干净的大脑  
  为什么啊，为什么是0啊，为什么啊  
  init限幅没改，我要爆炸了，好饿好困啊，早知道不玩小车了  
  出去走走，不然死在家里房子会贬值的。  
  看了一天手机，爱会消失对吗  ··
  ##  **2025.2.5**  
  知道为什么学不下去了，因为PID没思路，害怕了，问题在于不应该这么写三环，~~**学长给的三环应该是错的~~（置疑学长就是错的）或者说不适合轮腿，他那个是独轮车，我们轮腿应该参考最简单的平衡小车平衡就好了（速度，角度，转向），不能信学长😭  
  下午大改，先吃饭  
  堕落4天了，我发誓我明天再不狠狠学我让人欺负一辈子，臭手机NMSL。  
  ##  **2025.2.7**  
  事情又回到了起点，Pid变成了直立环，速度环，转向环，卡尔曼滤波又延迟太高，要么直接用学长的，要么再改这个网上抄的  
  好难啊，我应该如何学习下去，为什么只有我在学，我真的能接受队友只给钱不用学吗，为什么大佬进度那么快，一个字都写不下去，我知道现在我应该干嘛，但是我这么做如果不行怎么办，我又要重新找方法，为什么一个跟我一起学的都没有，所有方法都要我找，网上资料好少，所以我应该问问学长，老师？可是我明明还没到那种地步，我还可以靠自己排查解决问题，我现在问是不是太浪费了。
  目标：用最普通的平衡小车方法平衡，与网上的现象不同点在于：1pid函数不同，目前写了一个pid通用函数，似乎死区不同，应该重新写各自的函数。2，也是目前的最重大，最困难的一点，卡尔曼太慢了，可能原因不知道，放在主函数不行，会被中断打断？可能，放在1ms中断里还是太慢，需要5秒才能计算出目前角度，解决办法是要么就是这个卡尔曼就是个陷阱，算法无法优化，从新用学长的卡尔曼，要么就把这个卡尔曼改一下，emm,那个可能性更大呢，学长的优点是1可以问学长，肯定可以用，缺点是太乱，不知道怎么看，我应该自己找AI还是直接问学长，要么就用现在的这个卡尔曼，问题是这两个卡尔曼我都看不懂，只能用AI。我还是把这个卡尔曼仔细看看把，毕竟可以节省改框架的时间，既然两个都不懂的话，我还是寄希望于这个。好，所以我现在就要把卡尔曼喂给AI，然后问他为什么响应这么慢，改哪里才能让他响应快一点，如果能看得懂我就改，然后一点一点试，给我一天时间，一定不能急，别急，问题慢慢来会可以解决的，今天晚上加明天一天都搞这个卡尔曼，你别急，时间其实很充足，要慢慢来，要相信自己，今天晚上整不出来就先洗个澡，问题只是一个卡尔曼而已，整出来以后就没问题了，就可以直立了，一个阶段任务就基本完成了，所以现在的问题只有卡尔曼，只有这一个问题。   
  手好冷啊，我想先洗个澡，好难受  
  手好冷啊，
##  **2025.2.8**  
~~**问题无意间找到了，陀螺仪放置方向有问题导致角度解算慢，WTFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF，就这个问题搞得我，shittttttttttttttttttt,真是跟吃屎了一样**~~   
不对不对，他现在正常了？为什么跟昨天晚上状况不一样啊，wc,陀螺仪位置也不用改了？搞什么鬼  
还有7天开学，欸，如果我不刷视频会怎样，删掉B乎，b站，贴吧，小红书，来个7天实验，2.8下午4点开始  
离成功越来越近了，已经坐立难安  
## **2025.2.9**  
已经可以一直保持平衡了，再调一调就可以静止了，趁马上吃饭给你们总结一下吧  
~~按照时间顺序，先内环，用不着分两个轮子，测角度，然后输出putout,别忘了限幅不是0，角度环5ms,哪个环一开始都是先测极性，角度环先写一个p,慢慢加大，直到出现大幅低频振荡，这时候车平衡的趋势就很明显了，然后再加上d，慢慢加大，直到出现高频振荡，记得快点关不然驱动板会烧，这是得到的pd是直立环最大值，根据经验法，都乘0.6，然后调速度环PI，速度环中断时间就10ms,还有一点，所谓串级，算出来，就是并级，你可以在网上搜，串级打开来就是并级，先测极性，角度环系数都是0，选择一个轮子作为测速，转一下他应该会速度越来越大，把角度环系数写回来，这时候直接调P，I=p/200,调到你满意为止，ok,车就平衡了~~（2025.2.11全错，🤡哈哈哈，用串级）
就是一个平衡车，有了我这些经验，你们最多8天就能到这里了吧。我用18天😭😭😭  
##  **2025.2.10**  
放个假吧，车子太稳了，没事做了（2025.2.11学长的串级更稳）  
优化了结构，行走不行，一荡一荡的，原因不知道，初步解决办法是减小速度环系数，但是起不来，想法是mode切换，先起来stand，再walk.
##  **2025.2.11**  
~~**睡醒想明白了，初始状态，速度环使轮子向后转，车子前倾，角度环介入，车子向前走，速度增大，速度环退出，角度环介入，车子直立，速度为0；** ~~
**如果要保持平衡向前，那么小车就要保持一个恒定的倾角，使速度环向后输出而角度环向前输出，两个相互抵消在某一个值。**~~  
~~**所以荡是因为让速度环达到了他想要的值，他就不出力了，我们要么速度调的更大，要么让速度环变的无力（变小）。**~~  
笑死我了，全错全错，还是要听学长的，虽然你想不明白，但是听就完了，玩一天花了一晚上写串级pid,不仅更稳，而且能向前走了，芜湖呼呼呼，学长NB  
串级你就先把角速度环调到不高频振荡，再写角度环，可以写点I，再来点D，稍微有点就行，调角度环，先只调P，跳到很稳，然后用D消除稳态误差，然后小调速度环，使他尽量不动，几个系数极性一般是一样的，P多加D，D多加P，系数方面参考2.9写的  

