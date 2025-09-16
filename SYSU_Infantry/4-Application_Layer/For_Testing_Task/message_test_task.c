/**
 * @file    message_test_task.c
 * @brief   消息中心发布订阅模型测试任务源文件
 * @author  SYSU电控组
 * @date    2025-09-16
 * @version 1.0
 * 
 * @note    用于测试发布订阅模型的各种场景
 */

#include "message_test_task.h"
#include "message_center.h"
#include <stdio.h>
#include <string.h>

// 测试数据结构定义
typedef struct {
    uint32_t counter;
    float temperature;
    uint8_t status;
} sensor_data_t;

typedef struct {
    uint16_t motor_id;
    int16_t speed;
    int16_t current;
} motor_data_t;

typedef struct {
    uint8_t test_id;
    uint32_t timestamp;
    char message[16];
} test_message_t;

// 全局变量
static uint32_t test_counter = 0;

// 测试1：一对一发布订阅
static void Test_One_To_One(void)
{
    printf("\r\n=== Test 1: One Publisher -> One Subscriber ===\r\n");
    
    // 注册发布者和订阅者
    Publisher_t *pub = Pub_register("sensor_topic", sizeof(sensor_data_t));
    Subscriber_t *sub = Sub_register("sensor_topic", sizeof(sensor_data_t));
    
    if (pub == NULL || sub == NULL) {
        printf("Error: Registration failed\r\n");
        return;
    }
    
    // 发布数据
    sensor_data_t send_data = {
        .counter = test_counter,
        .temperature = 25.5f + (test_counter % 10),
        .status = 1
    };
    
    uint8_t result = Pub_push_message(pub, &send_data);
    printf("Published message to %d subscribers\r\n", result);
    
    // 接收数据
    sensor_data_t recv_data = {0};
    if (Sub_get_message(sub, &recv_data)) {
        printf("Received data: counter=%lu, temp=%.1f, status=%d\r\n", 
               recv_data.counter, recv_data.temperature, recv_data.status);
        
        // 验证数据一致性
        if (recv_data.counter == send_data.counter && 
            recv_data.temperature == send_data.temperature && 
            recv_data.status == send_data.status) {
            printf("✓ Data validation passed\r\n");
        } else {
            printf("✗ Data validation failed\r\n");
        }
    } else {
        printf("✗ No data received\r\n");
    }
}

// 测试2：多发布者对一订阅者
static void Test_Multi_Pub_One_Sub(void)
{
    printf("\r\n=== Test 2: Multiple Publishers -> One Subscriber ===\r\n");
    
    // 注册多个发布者（相同话题）
    Publisher_t *pub1 = Pub_register("motor_topic", sizeof(motor_data_t));
    Publisher_t *pub2 = Pub_register("motor_topic", sizeof(motor_data_t));
    
    // 注册一个订阅者
    Subscriber_t *sub = Sub_register("motor_topic", sizeof(motor_data_t));
    
    if (pub1 == NULL || pub2 == NULL || sub == NULL) {
        printf("Error: Registration failed\r\n");
        return;
    }
    
    // 检查是否是同一个发布者实例（应该是）
    if (pub1 == pub2) {
        printf("✓ Multiple publishers correctly share the same topic\r\n");
    } else {
        printf("✗ Multiple publishers do not share the topic correctly\r\n");
    }
    
    // 发布者1发布数据
    motor_data_t motor1_data = {
        .motor_id = 1,
        .speed = 1000,
        .current = 500
    };
    
    uint8_t result1 = Pub_push_message(pub1, &motor1_data);
    printf("Publisher 1 pushed message to %d subscribers\r\n", result1);
    
    // 接收第一条消息
    motor_data_t recv_data = {0};
    if (Sub_get_message(sub, &recv_data)) {
        printf("Received Publisher 1 data: motor_id=%d, speed=%d, current=%d\r\n", 
               recv_data.motor_id, recv_data.speed, recv_data.current);
    }
    
    // 发布者2发布数据（会覆盖之前的数据，因为QUEUE_SIZE=1）
    motor_data_t motor2_data = {
        .motor_id = 2,
        .speed = -800,
        .current = 300
    };
    
    uint8_t result2 = Pub_push_message(pub2, &motor2_data);
    printf("Publisher 2 pushed message to %d subscribers\r\n", result2);
    
    // 接收第二条消息
    memset(&recv_data, 0, sizeof(recv_data));
    if (Sub_get_message(sub, &recv_data)) {
        printf("Received Publisher 2 data: motor_id=%d, speed=%d, current=%d\r\n", 
               recv_data.motor_id, recv_data.speed, recv_data.current);
    } else {
        printf("✗ Did not receive Publisher 2 data\r\n");
    }
}

// 测试3：多发布者对多订阅者
static void Test_Multi_Pub_Multi_Sub(void)
{
    printf("\r\n=== Test 3: Multiple Publishers -> Multiple Subscribers ===\r\n");
    
    // 注册发布者
    Publisher_t *pub1 = Pub_register("test_topic", sizeof(test_message_t));
    Publisher_t *pub2 = Pub_register("test_topic", sizeof(test_message_t));
    
    // 注册多个订阅者
    Subscriber_t *sub1 = Sub_register("test_topic", sizeof(test_message_t));
    Subscriber_t *sub2 = Sub_register("test_topic", sizeof(test_message_t));
    Subscriber_t *sub3 = Sub_register("test_topic", sizeof(test_message_t));
    
    if (pub1 == NULL || pub2 == NULL || 
        sub1 == NULL || sub2 == NULL || sub3 == NULL) {
        printf("Error: Registration failed\r\n");
        return;
    }
    
    // 发布者1发布消息
    test_message_t msg1 = {
        .test_id = 1,
        .timestamp = test_counter,
    };
    strcpy(msg1.message, "Hello from P1");
    
    uint8_t result1 = Pub_push_message(pub1, &msg1);
    printf("Publisher 1 pushed message to %d subscribers\r\n", result1);
    
    // 所有订阅者接收消息
    test_message_t recv_msg = {0};
    for (int i = 1; i <= 3; i++) {
        Subscriber_t *sub = (i == 1) ? sub1 : (i == 2) ? sub2 : sub3;
        memset(&recv_msg, 0, sizeof(recv_msg));
        
        if (Sub_get_message(sub, &recv_msg)) {
            printf("Subscriber %d received: id=%d, time=%lu, msg='%s'\r\n", 
                   i, recv_msg.test_id, recv_msg.timestamp, recv_msg.message);
        } else {
            printf("✗ Subscriber %d did not receive message\r\n", i);
        }
    }
    
    // 发布者2发布消息
    test_message_t msg2 = {
        .test_id = 2,
        .timestamp = test_counter + 100,
    };
    strcpy(msg2.message, "Hello from P2");
    
    uint8_t result2 = Pub_push_message(pub2, &msg2);
    printf("Publisher 2 pushed message to %d subscribers\r\n", result2);
    
    // 所有订阅者接收新消息
    for (int i = 1; i <= 3; i++) {
        Subscriber_t *sub = (i == 1) ? sub1 : (i == 2) ? sub2 : sub3;
        memset(&recv_msg, 0, sizeof(recv_msg));
        
        if (Sub_get_message(sub, &recv_msg)) {
            printf("Subscriber %d received new message: id=%d, time=%lu, msg='%s'\r\n", 
                   i, recv_msg.test_id, recv_msg.timestamp, recv_msg.message);
        } else {
            printf("✗ Subscriber %d did not receive new message\r\n", i);
        }
    }
}

// 测试4：一发布者对多订阅者
static void Test_One_Pub_Multi_Sub(void)
{
    printf("\r\n=== Test 4: One Publisher -> Multiple Subscribers ===\r\n");
    
    // 注册一个发布者
    Publisher_t *pub = Pub_register("broadcast_topic", sizeof(sensor_data_t));
    
    // 注册多个订阅者
    Subscriber_t *sub1 = Sub_register("broadcast_topic", sizeof(sensor_data_t));
    Subscriber_t *sub2 = Sub_register("broadcast_topic", sizeof(sensor_data_t));
    Subscriber_t *sub3 = Sub_register("broadcast_topic", sizeof(sensor_data_t));
    Subscriber_t *sub4 = Sub_register("broadcast_topic", sizeof(sensor_data_t));
    
    if (pub == NULL || sub1 == NULL || sub2 == NULL || 
        sub3 == NULL || sub4 == NULL) {
        printf("Error: Registration failed\r\n");
        return;
    }
    
    // 发布广播消息
    sensor_data_t broadcast_data = {
        .counter = test_counter * 10,
        .temperature = 30.0f,
        .status = 0xFF
    };
    
    uint8_t result = Pub_push_message(pub, &broadcast_data);
    printf("Broadcast message to %d subscribers\r\n", result);
    
    // 验证所有订阅者都能接收到相同数据
    printf("Verifying all subscribers receive data:\r\n");
    
    Subscriber_t *subs[] = {sub1, sub2, sub3, sub4};
    for (int i = 0; i < 4; i++) {
        sensor_data_t recv_data = {0};
        if (Sub_get_message(subs[i], &recv_data)) {
            printf("Subscriber %d: counter=%lu, temp=%.1f, status=0x%02X ", 
                   i+1, recv_data.counter, recv_data.temperature, recv_data.status);
            
            // 验证数据一致性
            if (recv_data.counter == broadcast_data.counter && 
                recv_data.temperature == broadcast_data.temperature && 
                recv_data.status == broadcast_data.status) {
                printf("✓\r\n");
            } else {
                printf("✗\r\n");
            }
        } else {
            printf("Subscriber %d: ✗ No data received\r\n", i+1);
        }
    }
}

// 性能和并发测试
static void Test_Concurrent_Access(void)
{
    printf("\r\n=== Test 5: Concurrent Access Test ===\r\n");
    
    // 创建多个不同话题进行并发测试
    Publisher_t *pub_a = Pub_register("topic_a", sizeof(uint32_t));
    Publisher_t *pub_b = Pub_register("topic_b", sizeof(uint32_t));
    
    Subscriber_t *sub_a = Sub_register("topic_a", sizeof(uint32_t));
    Subscriber_t *sub_b = Sub_register("topic_b", sizeof(uint32_t));
    
    if (pub_a == NULL || pub_b == NULL || sub_a == NULL || sub_b == NULL) {
        printf("Error: Registration failed\r\n");
        return;
    }
    
    // 快速连续发布消息模拟并发
    uint32_t data_a = 0xAAAA0000 + test_counter;
    uint32_t data_b = 0xBBBB0000 + test_counter;
    
    uint8_t result_a = Pub_push_message(pub_a, &data_a);
    uint8_t result_b = Pub_push_message(pub_b, &data_b);
    
    printf("Topic A pushed to %d subscribers, Topic B pushed to %d subscribers\r\n", result_a, result_b);
    
    // 接收并验证数据
    uint32_t recv_a = 0, recv_b = 0;
    
    if (Sub_get_message(sub_a, &recv_a) && Sub_get_message(sub_b, &recv_b)) {
        printf("Received data - A: 0x%08lX, B: 0x%08lX\r\n", recv_a, recv_b);
        
        if (recv_a == data_a && recv_b == data_b) {
            printf("✓ Concurrent access test passed\r\n");
        } else {
            printf("✗ Concurrent access test failed\r\n");
        }
    } else {
        printf("✗ Concurrent access test - Data reception failed\r\n");
    }
}

/**
 * @brief 消息中心测试任务函数
 * @param argument 任务参数（未使用）
 */
void Message_test_task(void const *argument)
{
    // 等待系统初始化完成
    osDelay(2000);
    
    printf("\r\n========================================\r\n");
    printf("     Message Center Pub-Sub Model Test Started\r\n");
    printf("========================================\r\n");
    
    // 等待系统稳定
    osDelay(1000);
    
    for (;;)
    {
        printf("\r\n[Test Round %lu] Starting tests...\r\n", test_counter + 1);
        
        // 执行各项测试
        Test_One_To_One();
        osDelay(100);
        
        Test_Multi_Pub_One_Sub();
        osDelay(100);
        
        Test_Multi_Pub_Multi_Sub();
        osDelay(100);
        
        Test_One_Pub_Multi_Sub();
        osDelay(100);
        
        Test_Concurrent_Access();
        osDelay(100);
        
        test_counter++;
        
        printf("\r\n[Test Round %lu] Completed\r\n", test_counter);
        printf("========================================\r\n");
        
        // 每轮测试间隔5秒
        osDelay(5000);
    }
}
