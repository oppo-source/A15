#ifndef _OPPO_PROJECT_NEWCDT_H_
#define _OPPO_PROJECT_NEWCDT_H_

#define MAX_OCP 				6
#define MAX_LEN 				8
#define FEATURE_COUNT		10

#define ALIGN4(s) ((sizeof(s) + 3)&(~0x3))

enum OPPO_ENG_VERSION {
    RELEASE                 = 0x00,
    AGING                   = 0x01,
    CTA                     = 0x02,
    PERFORMANCE             = 0x03,
    PREVERSION              = 0x04,
    ALL_NET_CMCC_TEST       = 0x05,
    ALL_NET_CMCC_FIELD      = 0x06,
    ALL_NET_CU_TEST         = 0x07,
    ALL_NET_CU_FIELD        = 0x08,
    ALL_NET_CT_TEST         = 0x09,
    ALL_NET_CT_FIELD        = 0x0A,
};
/*chaibin@ODM_WT.BSP.Kernel.Boot, 2020/07/07, Add for devinfo*/
#ifdef ODM_WT_EDIT
enum OPPO_PROJECT_NEWCDT {
    OPPO_UNKOWN_NEWCDT = 0,
    OPPO_20091 = 20091,
    OPPO_20271 = 20271,
    OPPO_20272 = 20272,
    OPPO_20273 = 20273,
    OPPO_20274 = 20274,
    OPPO_2027A = 0x2027A,
    OPPO_2027B = 0x2027B,
    OPPO_2027C = 0x2027C,
    OPPO_2027D = 0x2027D,
    OPPO_2027E = 0x2027E,
};
#endif /* ODM_WT_EDIT*/

enum PCB_VERSION{
	PCB_UNKNOWN,
	PCB_VERSION_EVB1,
	PCB_VERSION_T0,
	PCB_VERSION_T1,
	PCB_VERSION_EVT1,
	PCB_VERSION_EVT2,
	PCB_VERSION_EVT3,
	PCB_VERSION_DVT1,
	PCB_VERSION_DVT2,
	PCB_VERSION_DVT3,
	PCB_VERSION_PVT1,
	PCB_VERSION_MP1,
	PCB_VERSION_MP2,
	PCB_VERSION_MP3,
	PCB_VERSION_MAX,
};

typedef struct
{
	unsigned int		nVersion;
	unsigned int		nProject;
	unsigned int		nDtsi;
	unsigned int		nAudio;
	unsigned int		nRF;
	unsigned int		nFeature[FEATURE_COUNT];
	unsigned int		nOppoBootMode;
	unsigned int		nPCB;
	unsigned int		newcdt;
	uint8_t			nPmicOcp[MAX_OCP];
	uint8_t			reserved[16];
} ProjectInfoCDTType;

typedef struct
{
  uint32_t   eng_version;
  uint32_t   is_confidential;
} EngInfoType;

unsigned int get_cdt_version_newcdt(void);
unsigned int get_project_newcdt(void);
unsigned int is_project_newcdt(int project);
unsigned int get_audio_project_newcdt(void);
unsigned int get_rf_info_newcdt(void);
unsigned int get_oppo_feature_newcdt(unsigned int feature_num);
unsigned int get_PCB_Version_newcdt(void);
int get_eng_version_newcdt(void);
int is_confidential_newcdt(void);
bool oppo_daily_build_newcdt(void);
int oppo_project_init_newcdt(void);
#endif
