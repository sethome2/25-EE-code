__packed typedef struct 
{
	float yaw,pitch;
	float x_speed,y_speed,r_speed;
	int bullet;
	int distance;
	float bullet_speed;
	
	int mode;
	char team;
	char unknow[27];
} STM32_data_t;


__packed typedef struct
{
	float yaw;
	float pitch;
	char shoot;
	char unknow[23];
} NUC_data_t;
extern NUC_data_t fromNUC;



//对STM32向NUC的信息解/编码
int encodeSTM32(STM32_data_t *target, unsigned char rx_buff[], unsigned int len);
int decodeSTM32(STM32_data_t *target, unsigned char tx_buff[], unsigned int len);

//对NUC向STM32的信息解/编码
int decodeNUC(NUC_data_t *target, unsigned char tx_buff[], unsigned int len);
int encodeNUC(NUC_data_t *target, unsigned char rx_buff[], unsigned int len);

//end of file
