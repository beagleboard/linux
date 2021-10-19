// SPDX-License-Identifier: GPL-2.0
/*
 * TI Bandgap temperature sensor driver for J72XX SoC Family
 *
 * Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/pm_runtime.h>
#include <linux/err.h>
#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/thermal.h>
#include <linux/of.h>
#include <linux/delay.h>

#define K3_VTM_DEVINFO_PWR0_OFFSET		0x4
#define K3_VTM_DEVINFO_PWR0_CVD_CT_MASK		0xf
#define K3_VTM_DEVINFO_PWR0_TEMPSENS_CT_MASK	0xf0
#define K3_VTM_TMPSENS0_CTRL_OFFSET		0x300
#define K3_VTM_MISC_CTRL_OFFSET			0xc
#define K3_VTM_TMPSENS_STAT_OFFSET		0x8
#define K3_VTM_ANYMAXT_OUTRG_ALERT_EN		0x1
#define K3_VTM_MISC_CTRL2_OFFSET		0x10
#define K3_VTM_REGS_PER_TS			0x10
#define K3_VTM_TS_STAT_DTEMP_MASK		0x3ff
#define K3_VTM_MAX_NUM_TS			8
#define K3_VTM_TMPSENS_CTRL_CBIASSEL		BIT(0)
#define K3_VTM_TMPSENS_CTRL_SOC			BIT(5)
#define K3_VTM_TMPSENS_CTRL_CLRZ		BIT(6)
#define K3_VTM_TMPSENS_CTRL_CLKON_REQ		BIT(7)
#define K3_VTM_TMPSENS_CTRL_MAXT_OUTRG_EN	BIT(11)

#define K3_VTM_CORRECTION_TEMP_CNT		3

#define K3_VTM_ADC_BEGIN_VAL			6
#define K3_VTM_ADC_END_VAL			944

#define MINUS40CREF				5
#define PLUS30CREF				253
#define PLUS125CREF				730
#define PLUS150CREF				940

#define MAX_TEMP				123000
#define COOL_DOWN_TEMP				105000

//#define DEBUG_VTM

static int derived_array[1024];

/* Reference Look up table when work around is NOT needed */
static const int pvt_poly_golden[] = {
	-49002, -48677, -48352, -48028, -47704, -47381, -47057, -46734, -46412,
	-46090, -45768, -45446, -45125, -44804, -44484, -44163, -43844, -43524,
	-43205, -42886, -42567, -42249, -41931, -41614, -41297, -40980, -40663,
	-40347, -40031, -39716, -39400, -39085, -38771, -38457, -38143, -37829,
	-37516, -37203, -36890, -36578, -36266, -35955, -35643, -35332, -35022,
	-34712, -34402, -34092, -33783, -33474, -33165, -32857, -32549, -32241,
	-31934, -31626, -31320, -31013, -30707, -30401, -30096, -29791, -29486,
	-29182, -28877, -28573, -28270, -27967, -27664, -27361, -27059, -26757,
	-26455, -26154, -25853, -25552, -25252, -24952, -24652, -24353, -24054,
	-23755, -23456, -23158, -22860, -22563, -22265, -21968, -21672, -21376,
	-21079, -20784, -20488, -20193, -19899, -19604, -19310, -19016, -18723,
	-18429, -18136, -17844, -17551, -17259, -16968, -16676, -16385, -16094,
	-15804, -15514, -15224, -14934, -14645, -14356, -14067, -13779, -13491,
	-13203, -12916, -12628, -12342, -12055, -11769, -11483, -11197, -10912,
	-10627, -10342, -10057, -9773, -9489, -9206, -8922, -8639, -8357, -8074,
	-7792, -7510, -7229, -6947, -6666, -6386, -6105, -5825, -5546, -5266,
	-4987, -4708, -4429, -4151, -3873, -3595, -3318, -3041, -2764, -2487,
	-2211, -1935, -1659, -1384, -1108, -834, -559, -285, -11, 263, 537, 810,
	1083, 1355, 1628, 1900, 2171, 2443, 2714, 2985, 3255, 3526, 3796, 4066,
	4335, 4604, 4873, 5142, 5410, 5679, 5946, 6214, 6481, 6748, 7015, 7281,
	7548, 7813, 8079, 8344, 8609, 8874, 9139, 9403, 9667, 9931, 10194,
	10457, 10720, 10983, 11245, 11507, 11769, 12030, 12292, 12553, 12813,
	13074, 13334, 13594, 13854, 14113, 14372, 14631, 14889, 15148, 15406,
	15664, 15921, 16178, 16435, 16692, 16948, 17205, 17461, 17716, 17972,
	18227, 18482, 18736, 18991, 19245, 19498, 19752, 20005, 20258, 20511,
	20764, 21016, 21268, 21520, 21771, 22022, 22273, 22524, 22774, 23025,
	23274, 23524, 23774, 24023, 24272, 24520, 24769, 25017, 25265, 25512,
	25760, 26007, 26254, 26500, 26747, 26993, 27239, 27484, 27730, 27975,
	28220, 28464, 28708, 28953, 29196, 29440, 29683, 29927, 30169, 30412,
	30654, 30896, 31138, 31380, 31621, 31862, 32103, 32344, 32584, 32824,
	33064, 33304, 33543, 33783, 34022, 34260, 34499, 34737, 34975, 35213,
	35450, 35687, 35924, 36161, 36397, 36634, 36870, 37105, 37341, 37576,
	37811, 38046, 38281, 38515, 38749, 38983, 39217, 39450, 39683, 39916,
	40149, 40381, 40614, 40846, 41077, 41309, 41540, 41771, 42002, 42233,
	42463, 42693, 42923, 43153, 43382, 43611, 43840, 44069, 44297, 44526,
	44754, 44981, 45209, 45436, 45663, 45890, 46117, 46343, 46570, 46796,
	47021, 47247, 47472, 47697, 47922, 48147, 48371, 48595, 48819, 49043,
	49266, 49490, 49713, 49935, 50158, 50380, 50603, 50824, 51046, 51268,
	51489, 51710, 51931, 52151, 52372, 52592, 52812, 53031, 53251, 53470,
	53689, 53908, 54127, 54345, 54563, 54781, 54999, 55217, 55434, 55651,
	55868, 56085, 56301, 56517, 56733, 56949, 57165, 57380, 57595, 57810,
	58025, 58239, 58454, 58668, 58882, 59095, 59309, 59522, 59735, 59948,
	60160, 60373, 60585, 60797, 61009, 61220, 61432, 61643, 61854, 62064,
	62275, 62485, 62695, 62905, 63115, 63324, 63534, 63743, 63952, 64160,
	64369, 64577, 64785, 64993, 65200, 65408, 65615, 65822, 66029, 66236,
	66442, 66648, 66854, 67060, 67266, 67471, 67676, 67881, 68086, 68291,
	68495, 68699, 68903, 69107, 69311, 69514, 69717, 69920, 70123, 70326,
	70528, 70730, 70932, 71134, 71336, 71537, 71738, 71939, 72140, 72341,
	72541, 72742, 72942, 73142, 73341, 73541, 73740, 73939, 74138, 74337,
	74535, 74733, 74932, 75130, 75327, 75525, 75722, 75919, 76116, 76313,
	76510, 76706, 76902, 77098, 77294, 77490, 77685, 77881, 78076, 78271,
	78465, 78660, 78854, 79048, 79242, 79436, 79630, 79823, 80016, 80210,
	80402, 80595, 80788, 80980, 81172, 81364, 81556, 81747, 81939, 82130,
	82321, 82512, 82702, 82893, 83083, 83273, 83463, 83653, 83843, 84032,
	84221, 84410, 84599, 84788, 84976, 85165, 85353, 85541, 85728, 85916,
	86103, 86291, 86478, 86665, 86851, 87038, 87224, 87411, 87597, 87782,
	87968, 88154, 88339, 88524, 88709, 88894, 89078, 89263, 89447, 89631,
	89815, 89999, 90182, 90366, 90549, 90732, 90915, 91098, 91280, 91463,
	91645, 91827, 92009, 92191, 92372, 92554, 92735, 92916, 93097, 93277,
	93458, 93638, 93818, 93998, 94178, 94358, 94537, 94717, 94896, 95075,
	95254, 95433, 95611, 95790, 95968, 96146, 96324, 96501, 96679, 96856,
	97033, 97211, 97387, 97564, 97741, 97917, 98093, 98269, 98445, 98621,
	98797, 98972, 99147, 99322, 99497, 99672, 99847, 100021, 100196, 100370,
	100544, 100718, 100891, 101065, 101238, 101411, 101584, 101757, 101930,
	102103, 102275, 102447, 102619, 102791, 102963, 103135, 103306, 103478,
	103649, 103820, 103991, 104161, 104332, 104502, 104673, 104843, 105013,
	105182, 105352, 105522, 105691, 105860, 106029, 106198, 106367, 106535,
	106704, 106872, 107040, 107208, 107376, 107544, 107711, 107878, 108046,
	108213, 108380, 108546, 108713, 108880, 109046, 109212, 109378, 109544,
	109710, 109875, 110041, 110206, 110371, 110536, 110701, 110866, 111030,
	111195, 111359, 111523, 111687, 111851, 112015, 112178, 112342, 112505,
	112668, 112831, 112994, 113157, 113319, 113482, 113644, 113806, 113968,
	114130, 114292, 114453, 114615, 114776, 114937, 115098, 115259, 115420,
	115580, 115741, 115901, 116061, 116221, 116381, 116541, 116701, 116860,
	117019, 117179, 117338, 117497, 117655, 117814, 117973, 118131, 118289,
	118447, 118605, 118763, 118921, 119078, 119236, 119393, 119550, 119707,
	119864, 120021, 120178, 120334, 120490, 120647, 120803, 120959, 121115,
	121270, 121426, 121581, 121737, 121892, 122047, 122202, 122356, 122511,
	122666, 122820, 122974, 123128, 123282, 123436, 123590, 123744, 123897,
	124050, 124204, 124357, 124510, 124662, 124815, 124968, 125120, 125273,
	125425, 125577, 125729, 125881, 126032, 126184, 126335, 126487, 126638,
	126789, 126940, 127091, 127241, 127392, 127542, 127693, 127843, 127993,
	128143, 128293, 128442, 128592, 128741, 128891, 129040, 129189, 129338,
	129487, 129635, 129784, 129932, 130081, 130229, 130377, 130525, 130673,
	130821, 130968, 131116, 131263, 131410, 131558, 131705, 131851, 131998,
	132145, 132291, 132438, 132584, 132730, 132876, 133022, 133168, 133314,
	133460, 133605, 133750, 133896, 134041, 134186, 134331, 134476, 134620,
	134765, 134909, 135054, 135198, 135342, 135486, 135630, 135773, 135917,
	136061, 136204, 136347, 136491, 136634, 136777, 136919, 137062, 137205,
	137347, 137490, 137632, 137774, 137916, 138058, 138200, 138342, 138483,
	138625, 138766, 138907, 139049, 139190, 139331, 139471, 139612, 139753,
	139893, 140034, 140174, 140314, 140454, 140594, 140734, 140874, 141013,
	141153, 141292, 141432, 141571, 141710, 141849, 141988, 142127, 142265,
	142404, 142542, 142681, 142819, 142957, 143095, 143233, 143371, 143509,
	143646, 143784, 143921, 144058, 144196, 144333, 144470, 144607, 144743,
	144880, 145017, 145153, 145289, 145426, 145562, 145698, 145834, 145970,
	146105, 146241, 146377, 146512, 146647, 146783, 146918, 147053, 147188,
	147323, 147457, 147592, 147727, 147861, 147995, 148130, 148264, 148398,
	148532, 148666, 148799, 148933, 149067, 149200, 149333, 149467, 149600,
	149733, 149866, 149999, 150131, 150264, 150397, 150529, 150661, 150794,
	150926, 151058, 151190, 151322, 151454, 151585, 151717, 151848, 151980,
	152111,	152242, 152373, 152505, 152635, 152766, 152897, 153028, 153158,
	153289, 153419, 153549, 153680, 153810, 153940, 154070, 154199, 154329,
	154459, 154588, 154718, 154847, 154976, 155105, 155235, 155363, 155492,
	155621, 155750, 155879, 156007, 156135, 156264, 156392, 156520, 156648,
	156776, 156904, 157032, 157160, 157287, 157415, 157542, 157670, 157797,
	157924, 158051, 158178, 158305, 158432, 158559, 158685, 158812, 158938,
	159065, 159191, 159317, 159443, 159569,
};

/* Reference Look up table when work around is needed */
static const int pvt_poly[] = {
	-41523, -41210, -40898, -40586, -40274, -39963, -39651, -39340,	-39029,
	-38718, -38408, -38098, -37788, -37478, -37168, -36859, -36550, -36241,
	-35933, -35624, -35316, -35008, -34701, -34393, -34086, -33779, -33472,
	-33166, -32859, -32553, -32247, -31942, -31636, -31331, -31026, -30722,
	-30417, -30113, -29809, -29505, -29202, -28899, -28596, -28293, -27990,
	-27688, -27386, -27084, -26782, -26481, -26179, -25878, -25578, -25277,
	-24977, -24677, -24377, -24078, -23778, -23479, -23180, -22882, -22583,
	-22285, -21987, -21689, -21392, -21095, -20797, -20501, -20204, -19908,
	-19612, -19316, -19020, -18725, -18430, -18135, -17840, -17545, -17251,
	-16957, -16663, -16370, -16077, -15783, -15491, -15198,	-14906, -14613,
	-14321, -14030, -13738, -13447, -13156, -12865, -12575, -12284, -11994,
	-11704, -11415, -11125, -10836, -10547, -10259, -9970, -9682, -9394,
	-9106, -8819, -8531, -8244, -7957, -7671, -7384, -7098, -6812, -6527,
	-6241, -5956, -5671, -5386, -5102, -4817, -4533, -4250, -3966, -3683,
	-3399, -3117, -2834, -2551, -2269, -1987, -1706, -1424, -1143, -862,
	-581, -300, -20, 260, 540, 820, 1099, 1378, 1657, 1936, 2215, 2493,
	2771, 3049, 3326, 3604, 3881, 4158, 4434, 4711, 4987, 5263, 5539, 5814,
	6089, 6364, 6639, 6914, 7188, 7462, 7736, 8010, 8283, 8556, 8829, 9102,
	9374, 9647, 9919, 10191, 10462, 10733, 11005, 11275, 11546, 11816,
	12087, 12357, 12626, 12896, 13165, 13434, 13703, 13971, 14240, 14508,
	14776, 15043, 15311, 15578, 15845, 16111, 16378, 16644, 16910, 17176,
	17441, 17707, 17972, 18237, 18501, 18766, 19030, 19294, 19557, 19821,
	20084, 20347, 20610, 20872, 21135, 21397, 21658, 21920, 22181, 22443,
	22703, 22964, 23225, 23485, 23745, 24005, 24264, 24523, 24782, 25041,
	25300, 25558, 25816, 26074, 26332, 26589, 26846, 27103, 27360, 27617,
	27873, 28129, 28385, 28640, 28896, 29151, 29406, 29660, 29915, 30169,
	30423, 30677, 30930, 31183, 31437, 31689, 31942, 32194, 32446, 32698,
	32950, 33201, 33452, 33703, 33954, 34205, 34455, 34705, 34955, 35204,
	35454, 35703, 35952, 36200, 36449, 36697, 36945, 37192, 37440, 37687,
	37934, 38181, 38427, 38674, 38920, 39166, 39411, 39657, 39902, 40147,
	40391, 40636, 40880, 41124, 41368, 41611, 41855, 42098, 42341, 42583,
	42826, 43068, 43310, 43551, 43793, 44034, 44275, 44516, 44756, 44997,
	45237, 45477, 45716, 45956, 46195, 46434, 46672, 46911, 47149, 47387,
	47625, 47862, 48100, 48337, 48573, 48810, 49046, 49282, 49518, 49754,
	49989, 50225, 50460, 50694, 50929, 51163, 51397, 51631, 51865, 52098,
	52331, 52564, 52797, 53029, 53261, 53493, 53725, 53956, 54188, 54419,
	54650, 54880, 55110, 55341, 55570, 55800, 56030, 56259, 56488, 56716,
	56945, 57173, 57401, 57629, 57856, 58084, 58311, 58538, 58764, 58991,
	59217, 59443, 59669, 59894, 60119, 60344, 60569, 60794, 61018, 61242,
	61466, 61690, 61913, 62136, 62359, 62582, 62804, 63026, 63249, 63470,
	63692, 63913, 64134, 64355, 64576, 64796, 65016, 65236, 65456, 65675,
	65894, 66113, 66332, 66551, 66769, 66987, 67205, 67423, 67640, 67857,
	68074, 68291, 68507, 68723, 68939, 69155, 69371, 69586, 69801, 70016,
	70230, 70445, 70659, 70873, 71086, 71300, 71513, 71726, 71939, 72151,
	72364, 72576, 72787, 72999, 73210, 73422, 73632, 73843, 74053, 74264,
	74474, 74683, 74893, 75102, 75311, 75520, 75729, 75937, 76145, 76353,
	76561, 76768, 76975, 77182, 77389, 77595, 77802, 78008, 78213, 78419,
	78624, 78829, 79034, 79239, 79443, 79647, 79851, 80055, 80259, 80462,
	80665, 80868, 81070, 81273, 81475, 81676, 81878, 82079, 82281, 82482,
	82682, 82883, 83083, 83283, 83483, 83682, 83882, 84081, 84280, 84478,
	84677, 84875, 85073, 85270, 85468, 85665, 85862, 86059, 86255, 86452,
	86648, 86844, 87039, 87234, 87430, 87625, 87819, 88014, 88208, 88402,
	88596, 88789, 88982, 89175, 89368, 89561, 89753, 89945, 90137, 90329,
	90520, 90712, 90903, 91093, 91284, 91474, 91664, 91854, 92044, 92233,
	92422, 92611, 92800, 92988, 93176, 93364, 93552, 93739, 93927, 94114,
	94301, 94487, 94673, 94860, 95045, 95231, 95417, 95602, 95787, 95971,
	96156, 96340, 96524, 96708, 96892, 97075, 97258, 97441, 97624, 97806,
	97988, 98170, 98352, 98533, 98714, 98895, 99076, 99257, 99437, 99617,
	99797, 99977, 100156, 100335, 100514, 100693, 100871, 101050, 101228,
	101405, 101583, 101760, 101937, 102114, 102291, 102467, 102643, 102819,
	102995, 103170, 103346, 103521, 103695, 103870, 104044, 104218, 104392,
	104566, 104739, 104912, 105085, 105258, 105430, 105603, 105775, 105946,
	106118, 106289, 106460, 106631, 106802, 106972, 107142, 107312, 107482,
	107651, 107820, 107989, 108158, 108327, 108495, 108663, 108831, 108998,
	109166, 109333, 109500, 109666, 109833, 109999, 110165, 110331, 110496,
	110661, 110827, 110991, 111156, 111320, 111484, 111648, 111812, 111975,
	112139, 112301, 112464, 112627, 112789, 112951, 113113, 113274, 113436,
	113597, 113758, 113918, 114079, 114239, 114399, 114559, 114718, 114877,
	115036, 115195, 115354, 115512, 115670, 115828, 115986, 116143, 116300,
	116457, 116614, 116770, 116926, 117082, 117238, 117394, 117549, 117704,
	117859, 118014, 118168, 118322, 118476, 118630, 118783, 118937, 119090,
	119242, 119395, 119547, 119699, 119851, 120003, 120154, 120305, 120456,
	120607, 120757, 120907, 121057, 121207, 121357, 121506, 121655, 121804,
	121953, 122101, 122249, 122397, 122545, 122692, 122839, 122986, 123133,
	123280, 123426, 123572, 123718, 123863, 124009, 124154, 124299, 124443,
	124588, 124732, 124876, 125020, 125163, 125306, 125449, 125592, 125735,
	125877, 126019, 126161, 126303, 126444, 126585, 126726, 126867, 127008,
	127148, 127288, 127428, 127567, 127707, 127846, 127985, 128123, 128262,
	128400, 128538, 128676, 128813, 128950, 129087, 129224, 129361, 129497,
	129633, 129769, 129905, 130040, 130175, 130310, 130445, 130579, 130713,
	130847, 130981, 131115, 131248, 131381, 131514, 131647, 131779, 131911,
	132043, 132175, 132306, 132438, 132569, 132699, 132830, 132960, 133090,
	133220, 133350, 133479, 133608, 133737, 133866, 133994, 134123, 134251,
	134378, 134506, 134633, 134760, 134887, 135014, 135140, 135266, 135392,
	135518, 135643, 135768, 135893, 136018, 136143, 136267, 136391, 136515,
	136638, 136762, 136885, 137008, 137130, 137253, 137375, 137497, 137619,
	137740, 137862, 137983, 138103, 138224, 138344, 138464, 138584, 138704,
	138823, 138943, 139062, 139180, 139299, 139417, 139535, 139653, 139770,
	139888, 140005, 140122, 140238, 140355, 140471, 140587, 140703, 140818,
	140933, 141048, 141163, 141278, 141392, 141506, 141620, 141734, 141847,
	141960, 142073, 142186, 142298, 142411, 142523, 142634, 142746, 142857,
	142968, 143079, 143190, 143300, 143410, 143520, 143630, 143740, 143849,
	143958, 144067, 144175, 144283, 144391, 144499, 144607, 144714, 144821,
	144928, 145035, 145142, 145248, 145354, 145459, 145565, 145670, 145775,
	145880, 145985, 146089, 146193, 146297, 146401, 146504, 146608, 146711,
	146813, 146916, 147018, 147120, 147222, 147324, 147425, 147526, 147627,
	147728, 147828, 147928, 148028, 148128, 148228, 148327, 148426, 148525,
	148623, 148722, 148820, 148918, 149015, 149113, 149210, 149307, 149404,
	149500, 149596, 149692, 149788, 149884, 149979, 150074, 150169, 150264,
	150358, 150452, 150546, 150640, 150733, 150827, 150920, 151013, 151105,
	151197, 151290, 151381, 151473, 151564, 151656, 151747, 151837, 151928,
	152018, 152108, 152198, 152287, 152377, 152466, 152555, 152643, 152732,
	152820, 152908, 152995, 153083, 153170, 153257, 153344, 153430, 153517,
	153603, 153689, 153774, 153860, 153945, 154030, 154114, 154199, 154283,
	154367, 154451, 154534, 154618, 154701, 154784, 154866, 154949, 155031,
	155113, 155194, 155276, 155357, 155438, 155519, 155599, 155680, 155760,
	155839, 155919, 155998, 156078, 156156, 156235, 156314, 156392, 156470,
	156547, 156625, 156702, 156779, 156856, 156933, 157009, 157085, 157161,
};

static void fill_table(int err1, int err2, int ref1, int ref2, int idx1,
		       int idx2)
{
	int m, c, num, den, i, err;

	/**
	 * Calculate the slope with adc values read from the register
	 * as the y-axis param and err in adc value as x-axis param
	 */
	if (err1 != err2) {
		num = ref2 - ref1;
		den = err2 - err1;
		m = num / den;
		c = ref2 - m * err2;
		for (i = idx1; i <= idx2; i++) {
			err = (i - c) / m;
			if (((i + err1) < 0) || ((i + err1) > 1023))
				continue;
			derived_array[i] = pvt_poly[i + err];
		}
	} else { /* Constant error take care of divide by zero */
		for (i = idx1; i < idx2; i++) {
			if (((i + err1) < 0) || ((i + err1) > 1023))
				continue;
			derived_array[i] = pvt_poly[i + err1];
		}
	}
}

static int prep_lookup_table(int minus40cerr, int plus30cerr, int plus125cerr,
			     int plus150cerr)
{
	int start_temp, inc, i;

	/* Fill up the lookup table region -40C to +30C */
	fill_table(minus40cerr, plus30cerr, MINUS40CREF, PLUS30CREF, 0,
		   PLUS30CREF);
	/* Fill up the lookup table region +30C to +125C */
	fill_table(plus30cerr, plus125cerr, PLUS30CREF, PLUS125CREF, PLUS30CREF,
		   PLUS125CREF);
	/* Fill up the lookup table region +125C to +150C */
	fill_table(plus125cerr, plus150cerr, PLUS125CREF, PLUS150CREF, PLUS125CREF,
		   PLUS150CREF);

	/* Get to the first valid temperature */
	i = 0;
	while (!derived_array[i])
		i++;

	start_temp = i;
	/*
	 * Get to the last zero index and back fill the temperature for
	 * sake of continuity
	 */
	if (i) {
		/* 300 milli celsius steps */
		while (i--)
			derived_array[i] = derived_array[i + 1] - 300;
		/* case 0 */
		derived_array[i] = derived_array[i + 1] - 300;
	}

	/*
	 * Fill the last trailing 0s which are unfilled with increments of
	 * 100 milli celsius till 1023 code
	 */
	i = 1023;
	while (!derived_array[i])
		i--;

	i++;
	inc = 1;
	while (i < 1024) {
		derived_array[i] = derived_array[i - 1] + inc * 100;
		i++;
	}

	return 0;
}

struct k3_thermal_data;

struct k3_j72xx_bandgap {
	struct device *dev;
	void __iomem *base;
	void __iomem *cfg2_base;
	void __iomem *fuse_base;
	const struct k3_j72xx_bandgap_data	*conf;
	spinlock_t lock; /* shields this struct */
	int ts_cnt;
	struct k3_thermal_data *ts_data[K3_VTM_MAX_NUM_TS];
};

/* common data structures */
struct k3_thermal_data {
	struct thermal_zone_device *ti_thermal;
	struct thermal_cooling_device *cool_dev;
	struct k3_j72xx_bandgap *bgp;
	enum thermal_device_mode mode;
	int sensor_id;
	u32 ctrl_offset;
	u32 stat_offset;
	int prev_temp;
	int ct_offsets[K3_VTM_CORRECTION_TEMP_CNT];
	int ct_bm[K3_VTM_CORRECTION_TEMP_CNT];
	int ref_adc_val[3];
	int act_adc_val[3];
	int slope[2];
	int coeff[2];
};

int two_cmp(int tmp, int mask)
{
	tmp = ~(tmp);
	tmp &= mask;
	tmp += 1;

	/* Return negative value */
	return (0 - tmp);
}

static unsigned int vtm_get_best_value(unsigned int s0, unsigned int s1,
				       unsigned int s2)
{
	int d01 = abs(s0 - s1);
	int d02 = abs(s0 - s2);
	int d12 = abs(s1 - s2);

	if (d01 <= d02 && d01 <= d12)
		return (s0 + s1) / 2;

	if (d02 <= d01 && d02 <= d12)
		return (s0 + s2) / 2;

	return (s1 + s2) / 2;
}

static inline int k3_bgp_read_temp(struct k3_thermal_data *devdata,
				   int *temp)
{
	struct k3_j72xx_bandgap *bgp;
	unsigned int dtemp, s0, s1, s2;

	bgp = devdata->bgp;
	/*
	 * Errata is applicable for am654 pg 1.0 silicon/J7ES. There
	 * is a variation of the order for certain degree centigrade on AM654.
	 * Work around that by getting the average of two closest
	 * readings out of three readings everytime we want to
	 * report temperatures.
	 *
	 * Errata workaround.
	 */
	s0 = readl(bgp->base + devdata->stat_offset) &
		K3_VTM_TS_STAT_DTEMP_MASK;
	s1 = readl(bgp->base + devdata->stat_offset) &
		K3_VTM_TS_STAT_DTEMP_MASK;
	s2 = readl(bgp->base + devdata->stat_offset) &
		K3_VTM_TS_STAT_DTEMP_MASK;
	dtemp = vtm_get_best_value(s0, s1, s2);

	if (dtemp < 0 || dtemp > 1023)
		return -EINVAL;

	*temp = derived_array[dtemp];

	return 0;
}

/* Get temperature callback function for thermal zone */
static int k3_thermal_get_temp(void *devdata, int *temp)
{
	struct k3_thermal_data *data = devdata;
	int ret = 0;

	ret = k3_bgp_read_temp(data, temp);
	if (ret)
		return ret;

	data->prev_temp = *temp;

	return ret;
}

static const struct thermal_zone_of_device_ops k3_of_thermal_ops = {
	.get_temp = k3_thermal_get_temp,
};

static int k3_j72xx_bandgap_temp_to_adc_code(int temp)
{
	int low = 0, high = 1023, mid;

	if (temp > 160000 || temp < -50000)
		return -EINVAL;

	/* Binary search to find the adc code */
	while (low < (high - 1)) {
		mid = (low + high) / 2;
		if (temp <= derived_array[mid])
			high = mid;
		else
			low = mid;
	}

	return mid;
}

static void get_efuse_values(int id, struct k3_thermal_data *data, int *err,
			     struct k3_j72xx_bandgap *bgp)
{
	int i, tmp, pow;

	/* Populate efuse reg offsets & Bit masks for -40C, 30C, 125C */
	switch (id) {
	case 0:
		data->ct_offsets[0] = 0x0;
		data->ct_offsets[1] = 0x8;
		data->ct_offsets[2] = 0x4;
		data->ct_bm[0] = 0x3f;
		data->ct_bm[1] = 0x1fe000;
		data->ct_bm[2] = 0x1ff;
		break;

	case 1:
		data->ct_offsets[0] = 0x0;
		data->ct_offsets[1] = 0x8;
		data->ct_offsets[2] = 0x4;
		data->ct_bm[0] = 0xfc0;
		data->ct_bm[1] = 0x1fe00000;
		data->ct_bm[2] = 0x3fe00;
		break;

	case 2:
		data->ct_offsets[0] = 0x0;
		data->ct_offsets[1] = -1;
		data->ct_offsets[2] = 0x4;
		data->ct_bm[0] = 0x3f000;
		data->ct_bm[1] = 0x7f800000;
		data->ct_bm[2] = 0x7fc0000;
		break;

	case 3:
		data->ct_offsets[0] = 0x0;
		data->ct_offsets[1] = 0xC;
		data->ct_offsets[2] = -1; /* Spread across 2 registers */
		data->ct_bm[0] = 0xfc0000;
		data->ct_bm[1] = 0x1fe0;
		data->ct_bm[2] = 0x1f800000;
		break;

	case 4:
		data->ct_offsets[0] = 0x0;
		data->ct_offsets[1] = 0xc;
		data->ct_offsets[2] = 0x8;
		data->ct_bm[0] = 0x3f000000;
		data->ct_bm[1] = 0x1fe000;
		data->ct_bm[2] = 0x1ff0;
		break;
	}

	for (i = 0; i < 3; i++) {
		/* Extract the offset value using bit-mask */
		if (data->ct_offsets[i] == -1 && i == 1) {
			/* 25C offset Case of Sensor 2 split between 2 regs */
			tmp = (readl(bgp->fuse_base + 0x8) & 0xE0000000) >> (29);
			tmp |= ((readl(bgp->fuse_base + 0xC) & 0x1F) << 3);
			pow = tmp & 0x80;
		} else if (data->ct_offsets[i] == -1 && i == 2) {
			/* 125C Case of Sensor 3 split between 2 regs */
			tmp = (readl(bgp->fuse_base + 0x4) & 0xF8000000) >> (27);
			tmp |= ((readl(bgp->fuse_base + 0x8) & 0xF) << 5);
			pow = tmp & 0x100;
		} else {
			tmp = readl(bgp->fuse_base + data->ct_offsets[i]);
			tmp &= data->ct_bm[i];
			tmp = tmp >> __ffs(data->ct_bm[i]);

			/* Obtain the sign bit pow*/
			pow = data->ct_bm[i] >> __ffs(data->ct_bm[i]);
			pow += 1;
			pow /= 2;
		}

		/* Check for negative value */
		if (tmp & pow) {
			/* 2's complement value */
			tmp = two_cmp(tmp, data->ct_bm[i] >> __ffs(data->ct_bm[i]));
		}
		err[i] = tmp;
	}
}

#ifdef DEBUG_VTM
static void print_look_up_table(struct device *dev)
{
	int i;

	dev_info(dev, "The contents of derived array\n");
	dev_info(dev, "Code   Temperaturei\n");
	for (i = 0; i <= 1023; i++)
		dev_info(dev, "%d       %d\n", i, derived_array[i]);
}
#endif

static const struct of_device_id of_k3_j72xx_bandgap_match[];

struct k3_j72xx_bandgap_j72xx_data {
	unsigned int	workaround;
};

static int k3_j72xx_bandgap_probe(struct platform_device *pdev)
{
	int ret = 0, cnt, val, id, reg_cnt = 0, j;
	int err[3], high_max, low_max;
	struct resource *res;
	struct device *dev = &pdev->dev;
	struct k3_j72xx_bandgap *bgp;
	struct k3_thermal_data *data;
	int workaround_needed = 0;
	const struct of_device_id *of_id;
	const struct k3_j72xx_bandgap_j72xx_data *driver_data;

	bgp = devm_kzalloc(&pdev->dev, sizeof(*bgp), GFP_KERNEL);
	if (!bgp)
		return -ENOMEM;

	bgp->dev = dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	bgp->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(bgp->base))
		return PTR_ERR(bgp->base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	bgp->cfg2_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(bgp->cfg2_base))
		return PTR_ERR(bgp->cfg2_base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	bgp->fuse_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(bgp->fuse_base))
		return PTR_ERR(bgp->fuse_base);

	of_id = of_match_device(of_k3_j72xx_bandgap_match, &pdev->dev);
	if (of_id) {
		driver_data = of_id->data;
		workaround_needed = driver_data->workaround;
	}

	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		pm_runtime_put_noidle(dev);
		pm_runtime_disable(dev);
		return ret;
	}

	/* Get the sensor count in the VTM */
	val = readl(bgp->base + K3_VTM_DEVINFO_PWR0_OFFSET);
	cnt = val & K3_VTM_DEVINFO_PWR0_TEMPSENS_CT_MASK;
	cnt >>= __ffs(K3_VTM_DEVINFO_PWR0_TEMPSENS_CT_MASK);
	bgp->ts_cnt = cnt;

	data = devm_kcalloc(bgp->dev, cnt, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto err_alloc;
	}

	/* Workaround not needed if bit30/bit31 is set even for J721e */
	if (workaround_needed && (readl(bgp->fuse_base + 0x0) & 0xc0000000) == 0xc0000000) {
		dev_info(bgp->dev, "work around NOT needed!!\n");
		workaround_needed = 0;
	}

	/* Register the thermal sensors */
	for (id = 0; id < cnt; id++) {
		data[id].sensor_id = id;
		data[id].bgp = bgp;
		data[id].ctrl_offset = K3_VTM_TMPSENS0_CTRL_OFFSET + id * 0x20;
		data[id].stat_offset = data[id].ctrl_offset +
					K3_VTM_TMPSENS_STAT_OFFSET;

		if (!workaround_needed)
			goto prepare_lookup;
		/* ref adc values for -40C, 30C & 125C respectively */
		data[id].ref_adc_val[0] = MINUS40CREF;
		data[id].ref_adc_val[1] = PLUS30CREF;
		data[id].ref_adc_val[2] = PLUS125CREF;
		get_efuse_values(id, &data[id], err, bgp);

prepare_lookup:
		if (id == 0 && workaround_needed) {
			prep_lookup_table(err[0], err[1], err[2], 0);
		} else if (id == 0 && !workaround_needed) {
			for (j = 0; j < (sizeof(pvt_poly_golden) / 4); j++)
				derived_array[j] = pvt_poly_golden[j];
		}

		val = readl(data[id].bgp->cfg2_base + data[id].ctrl_offset);
		val |= (K3_VTM_TMPSENS_CTRL_MAXT_OUTRG_EN |
			K3_VTM_TMPSENS_CTRL_SOC |
			K3_VTM_TMPSENS_CTRL_CLRZ | BIT(4));
		writel(val, data[id].bgp->cfg2_base + data[id].ctrl_offset);

		bgp->ts_data[id] = &data[id];
		data[id].ti_thermal =
		devm_thermal_zone_of_sensor_register(bgp->dev, id,
						     &data[id],
						     &k3_of_thermal_ops);
		if (IS_ERR(data[id].ti_thermal)) {
			dev_err(bgp->dev, "thermal zone device is NULL\n");
			ret = PTR_ERR(data[id].ti_thermal);
			goto err_alloc;
		}

		reg_cnt++;

		/* Initialize Previous temp */
		k3_thermal_get_temp(&data[id], &data[id].prev_temp);
	}

	/*
	 * Program TSHUT thresholds
	 * Step 1: set the thresholds to ~123C and 105C WKUP_VTM_MISC_CTRL2
	 * Step 2: WKUP_VTM_TMPSENS_CTRL_j set the MAXT_OUTRG_EN  bit
	 *         This is already taken care as per of init
	 * Step 3: WKUP_VTM_MISC_CTRL set the ANYMAXT_OUTRG_ALERT_EN  bit
	 */
	high_max = k3_j72xx_bandgap_temp_to_adc_code(MAX_TEMP);
	low_max = k3_j72xx_bandgap_temp_to_adc_code(COOL_DOWN_TEMP);

	writel((low_max << 16) | high_max, data[0].bgp->cfg2_base +
	       K3_VTM_MISC_CTRL2_OFFSET);
	mdelay(100);
	writel(K3_VTM_ANYMAXT_OUTRG_ALERT_EN, data[0].bgp->cfg2_base +
	       K3_VTM_MISC_CTRL_OFFSET);

	platform_set_drvdata(pdev, bgp);

#ifdef DEBUG_VTM
	print_look_up_table(dev);
#endif
	return 0;

err_alloc:
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return ret;
}

static int k3_j72xx_bandgap_remove(struct platform_device *pdev)
{
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

const struct k3_j72xx_bandgap_j72xx_data k3_j72xx_bandgap_j721e_data = {
	.workaround = 1,
};

const struct k3_j72xx_bandgap_j72xx_data k3_j72xx_bandgap_j7200_data = {
	.workaround = 0,
};

static const struct of_device_id of_k3_j72xx_bandgap_match[] = {
	{
		.compatible = "ti,j721e-vtm",
		.data = (void *)&k3_j72xx_bandgap_j721e_data,
	},
	{
		.compatible = "ti,j7200-vtm",
		.data = (void *)&k3_j72xx_bandgap_j7200_data,
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, of_k3_j72xx_bandgap_match);

static struct platform_driver k3_j72xx_bandgap_sensor_driver = {
	.probe = k3_j72xx_bandgap_probe,
	.remove = k3_j72xx_bandgap_remove,
	.driver = {
		.name = "k3-j72xx-soc-thermal",
		.of_match_table	= of_k3_j72xx_bandgap_match,
	},
};

module_platform_driver(k3_j72xx_bandgap_sensor_driver);

MODULE_DESCRIPTION("K3 bandgap temperature sensor driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("J Keerthy <j-keerthy@ti.com>");
