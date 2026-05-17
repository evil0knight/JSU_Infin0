/*
 * flash.c
 *
 *  Created on: 2024年11月25日
 *      Author: 17104
 */

#include <database.h>

//二级缓冲区二维数组声明
//示例:flash_data_union database_buffer[12][data_num * 2];

//数据库清空
//示例：clear_all_data();
void clear_all_data(void)
{
    int i;
    for(i = 0; i < 12; i++)
    {
        if(flash_check(0,i))
        {
            flash_erase_page(0, i);
        }
    }
}

//写入数据0-整型 1-浮点型
//示例：float a = 3.1415f;uint8 status = write_database(0,0,&a,1);
//1-表示失败 0-表示成功
uint8 write_database(uint32 page_num,int order_num,void * data,int is_float)
{
    flash_buffer_clear();
    uint8 status = 0;
    int i;
    if(is_float == 1)
    {
        float *p = data;
        database_buffer[page_num][order_num * 2].float_type = *p;
        database_buffer[page_num][order_num * 2 + 1].int32_type = 1;

        for(i = 0; i < data_num * 2;i++)
        {
            flash_union_buffer[i].int32_type = database_buffer[page_num][i].int32_type;
        }
        status = flash_write_page_from_buffer(0, page_num);
    }
    else if(is_float == 0)
    {
        int32 *p = data;
        database_buffer[page_num][order_num * 2].int32_type = *p;
        database_buffer[page_num][order_num * 2 + 1].int32_type = 0;
        for(i = 0; i < data_num * 2;i++)
        {
            flash_union_buffer[i].int32_type = database_buffer[page_num][i].int32_type;
        }
        status = flash_write_page_from_buffer(0, page_num);
    }
    else
    {
        status = 1;
    }

    return status;
}

//获取浮点型数据
//示例：float output = read_database_float(0,0);
//250-表示失败
float read_database_float(uint32 page_num,int order_num)
{
    flash_buffer_clear();
    flash_read_page_to_buffer(0, page_num);
    int i;
    for(i = 0; i < data_num * 2;i++)
    {
        database_buffer[page_num][i].int32_type = flash_union_buffer[i].int32_type;
    }
    if(database_buffer[page_num][order_num * 2 + 1].int32_type)
    {
        return database_buffer[page_num][order_num * 2].float_type;
    }
    else
    {
        return 250;
    }

}

//获取整型数据
//示例：int output = read_database_int(0,0);
//250-表示失败
int read_database_int(uint32 page_num,int order_num)
{
    flash_buffer_clear();
    flash_read_page_to_buffer(0, page_num);
    int i;
    for(i = 0; i < data_num * 2;i++)
    {
        database_buffer[page_num][i].int32_type = flash_union_buffer[i].int32_type;
    }
    if(!database_buffer[page_num][order_num * 2 + 1].int32_type)
    {
        return database_buffer[page_num][order_num * 2].int32_type;
    }
    else
    {
        return 250;
    }

}
