/*
 *  Sysfs interface for the universal power supply monitor class
 *
 *  Copyright © 2007  David Woodhouse <dwmw2@infradead.org>
 *  Copyright © 2007  Anton Vorontsov <cbou@mail.ru>
 *  Copyright © 2004  Szabolcs Gyurko
 *  Copyright © 2003  Ian Molton <spyro@f2s.com>
 *
 *  Modified: 2004, Oct     Szabolcs Gyurko
 *
 *  You may use this code as per GPL version 2
 */

#include <linux/ctype.h>
#include <linux/device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/stat.h>

#include "power_supply.h"

#define MAX_PROP_NAME_LEN 30

struct power_supply_attr {
	const char *prop_name;
	char attr_name[MAX_PROP_NAME_LEN + 1];
	struct device_attribute dev_attr;
	const char * const *text_values;
	int text_values_len;
};

#define _POWER_SUPPLY_ATTR(_name, _text, _len)	\
[POWER_SUPPLY_PROP_ ## _name] =			\
{						\
	.prop_name = #_name,			\
	.attr_name = #_name "\0",		\
	.text_values = _text,			\
	.text_values_len = _len,		\
}

#define POWER_SUPPLY_ATTR(_name) _POWER_SUPPLY_ATTR(_name, NULL, 0)
#define _POWER_SUPPLY_ENUM_ATTR(_name, _text)	\
	_POWER_SUPPLY_ATTR(_name, _text, ARRAY_SIZE(_text))
#define POWER_SUPPLY_ENUM_ATTR(_name)	\
	_POWER_SUPPLY_ENUM_ATTR(_name, POWER_SUPPLY_ ## _name ## _TEXT)

static const char * const POWER_SUPPLY_TYPE_TEXT[] = {
	[POWER_SUPPLY_TYPE_UNKNOWN]		= "Unknown",
	[POWER_SUPPLY_TYPE_BATTERY]		= "Battery",
	[POWER_SUPPLY_TYPE_UPS]			= "UPS",
	[POWER_SUPPLY_TYPE_MAINS]		= "Mains",
	[POWER_SUPPLY_TYPE_USB]			= "USB",
	[POWER_SUPPLY_TYPE_USB_DCP]		= "USB_DCP",
	[POWER_SUPPLY_TYPE_USB_CDP]		= "USB_CDP",
	[POWER_SUPPLY_TYPE_USB_ACA]		= "USB_ACA",
	[POWER_SUPPLY_TYPE_USB_TYPE_C]		= "USB_C",
	[POWER_SUPPLY_TYPE_USB_PD]		= "USB_PD",
	[POWER_SUPPLY_TYPE_USB_PD_DRP]		= "USB_PD_DRP",
	[POWER_SUPPLY_TYPE_APPLE_BRICK_ID]	= "BrickID",
	[POWER_SUPPLY_TYPE_USB_HVDCP]		= "USB_HVDCP",
	[POWER_SUPPLY_TYPE_USB_HVDCP_3]		= "USB_HVDCP_3",
	[POWER_SUPPLY_TYPE_USB_HVDCP_3P5]	= "USB_HVDCP_3P5",
	[POWER_SUPPLY_TYPE_WIRELESS]		= "Wireless",
	[POWER_SUPPLY_TYPE_USB_FLOAT]		= "USB_FLOAT",
	[POWER_SUPPLY_TYPE_BMS]			= "BMS",
	[POWER_SUPPLY_TYPE_PARALLEL]		= "Parallel",
	[POWER_SUPPLY_TYPE_MAIN]		= "Main",
	[POWER_SUPPLY_TYPE_UFP]			= "USB_C_UFP",
	[POWER_SUPPLY_TYPE_DFP]			= "USB_C_DFP",
	[POWER_SUPPLY_TYPE_CHARGE_PUMP]		= "Charge_Pump",
};

static const char * const POWER_SUPPLY_USB_TYPE_TEXT[] = {
	[POWER_SUPPLY_USB_TYPE_UNKNOWN]		= "Unknown",
	[POWER_SUPPLY_USB_TYPE_SDP]		= "SDP",
	[POWER_SUPPLY_USB_TYPE_DCP]		= "DCP",
	[POWER_SUPPLY_USB_TYPE_CDP]		= "CDP",
	[POWER_SUPPLY_USB_TYPE_ACA]		= "ACA",
	[POWER_SUPPLY_USB_TYPE_C]		= "C",
	[POWER_SUPPLY_USB_TYPE_PD]		= "PD",
	[POWER_SUPPLY_USB_TYPE_PD_DRP]		= "PD_DRP",
	[POWER_SUPPLY_USB_TYPE_PD_PPS]		= "PD_PPS",
	[POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID]	= "BrickID",
};

static const char * const POWER_SUPPLY_STATUS_TEXT[] = {
	[POWER_SUPPLY_STATUS_UNKNOWN]		= "Unknown",
	[POWER_SUPPLY_STATUS_CHARGING]		= "Charging",
	[POWER_SUPPLY_STATUS_DISCHARGING]	= "Discharging",
	[POWER_SUPPLY_STATUS_NOT_CHARGING]	= "Not charging",
	[POWER_SUPPLY_STATUS_FULL]		= "Full",
};

static const char * const POWER_SUPPLY_CHARGE_TYPE_TEXT[] = {
	[POWER_SUPPLY_CHARGE_TYPE_UNKNOWN]	= "Unknown",
	[POWER_SUPPLY_CHARGE_TYPE_NONE]		= "N/A",
	[POWER_SUPPLY_CHARGE_TYPE_TRICKLE]	= "Trickle",
	[POWER_SUPPLY_CHARGE_TYPE_FAST]		= "Fast",
	[POWER_SUPPLY_CHARGE_TYPE_TAPER]	= "Taper",
};

static const char * const POWER_SUPPLY_HEALTH_TEXT[] = {
	[POWER_SUPPLY_HEALTH_UNKNOWN]		    = "Unknown",
	[POWER_SUPPLY_HEALTH_GOOD]		    = "Good",
	[POWER_SUPPLY_HEALTH_OVERHEAT]		    = "Overheat",
	[POWER_SUPPLY_HEALTH_DEAD]		    = "Dead",
	[POWER_SUPPLY_HEALTH_OVERVOLTAGE]	    = "Over voltage",
	[POWER_SUPPLY_HEALTH_UNSPEC_FAILURE]	    = "Unspecified failure",
	[POWER_SUPPLY_HEALTH_COLD]		    = "Cold",
	[POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE] = "Watchdog timer expire",
	[POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE]   = "Safety timer expire",
	[POWER_SUPPLY_HEALTH_OVERCURRENT]	    = "Over current",
	[POWER_SUPPLY_HEALTH_CALIBRATION_REQUIRED]  = "Calibration required",
	[POWER_SUPPLY_HEALTH_WARM]		    = "Warm",
	[POWER_SUPPLY_HEALTH_COOL]		    = "Cool",
	[POWER_SUPPLY_HEALTH_HOT]		    = "Hot",
};

static const char * const POWER_SUPPLY_TECHNOLOGY_TEXT[] = {
	[POWER_SUPPLY_TECHNOLOGY_UNKNOWN]	= "Unknown",
	[POWER_SUPPLY_TECHNOLOGY_NiMH]		= "NiMH",
	[POWER_SUPPLY_TECHNOLOGY_LION]		= "Li-ion",
	[POWER_SUPPLY_TECHNOLOGY_LIPO]		= "Li-poly",
	[POWER_SUPPLY_TECHNOLOGY_LiFe]		= "LiFe",
	[POWER_SUPPLY_TECHNOLOGY_NiCd]		= "NiCd",
	[POWER_SUPPLY_TECHNOLOGY_LiMn]		= "LiMn",
};

static const char * const POWER_SUPPLY_CAPACITY_LEVEL_TEXT[] = {
	[POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN]	= "Unknown",
	[POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL]	= "Critical",
	[POWER_SUPPLY_CAPACITY_LEVEL_LOW]	= "Low",
	[POWER_SUPPLY_CAPACITY_LEVEL_NORMAL]	= "Normal",
	[POWER_SUPPLY_CAPACITY_LEVEL_HIGH]	= "High",
	[POWER_SUPPLY_CAPACITY_LEVEL_FULL]	= "Full",
};

static const char * const POWER_SUPPLY_SCOPE_TEXT[] = {
	[POWER_SUPPLY_SCOPE_UNKNOWN]	= "Unknown",
	[POWER_SUPPLY_SCOPE_SYSTEM]	= "System",
	[POWER_SUPPLY_SCOPE_DEVICE]	= "Device",
};

static const char * const POWER_SUPPLY_TYPEC_MODE_TEXT[] = {
	[POWER_SUPPLY_TYPEC_NONE] 			= "Nothing attached",
	[POWER_SUPPLY_TYPEC_SINK] 			= "Sink attached",
	[POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE] 	= "Powered cable w/ sink",
	[POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY] 	= "Debug Accessory",
	[POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER] 	= "Audio Adapter",
	[POWER_SUPPLY_TYPEC_POWERED_CABLE_ONLY] 	= "Powered cable w/o sink",
	[POWER_SUPPLY_TYPEC_SOURCE_DEFAULT] 		= "Source attached (default current)",
	[POWER_SUPPLY_TYPEC_SOURCE_MEDIUM] 		= "Source attached (medium current)",
	[POWER_SUPPLY_TYPEC_SOURCE_HIGH] 		= "Source attached (high current)",
	[POWER_SUPPLY_TYPEC_NON_COMPLIANT] 		= "Non compliant",
};

static const char * const POWER_SUPPLY_TYPEC_POWER_ROLE_TEXT[] = {
	[POWER_SUPPLY_TYPEC_PR_NONE] 	= "none",
	[POWER_SUPPLY_TYPEC_PR_DUAL] 	= "dual power role",
	[POWER_SUPPLY_TYPEC_PR_SINK] 	= "sink",
	[POWER_SUPPLY_TYPEC_PR_SOURCE] 	= "source",
};

static const char * const POWER_SUPPLY_TYPEC_SRC_RP_TEXT[] = {
	[POWER_SUPPLY_TYPEC_SRC_RP_STD] 	= "Rp-Default",
	[POWER_SUPPLY_TYPEC_SRC_RP_1P5A] 	= "Rp-1.5A",
	[POWER_SUPPLY_TYPEC_SRC_RP_3A] 		= "Rp-3A",
};

static struct power_supply_attr power_supply_attrs[] = {
	/* Properties of type `int' */
	POWER_SUPPLY_ENUM_ATTR(STATUS),
	POWER_SUPPLY_ENUM_ATTR(CHARGE_TYPE),
	POWER_SUPPLY_ENUM_ATTR(HEALTH),
	POWER_SUPPLY_ATTR(PRESENT),
	POWER_SUPPLY_ATTR(ONLINE),
	POWER_SUPPLY_ATTR(AUTHENTIC),
	POWER_SUPPLY_ENUM_ATTR(TECHNOLOGY),
	POWER_SUPPLY_ATTR(CYCLE_COUNT),
	POWER_SUPPLY_ATTR(VOLTAGE_MAX),
	POWER_SUPPLY_ATTR(VOLTAGE_MIN),
	POWER_SUPPLY_ATTR(VOLTAGE_MAX_DESIGN),
	POWER_SUPPLY_ATTR(VOLTAGE_MIN_DESIGN),
	POWER_SUPPLY_ATTR(VOLTAGE_NOW),
	POWER_SUPPLY_ATTR(VOLTAGE_AVG),
	POWER_SUPPLY_ATTR(VOLTAGE_OCV),
	POWER_SUPPLY_ATTR(VOLTAGE_BOOT),
	POWER_SUPPLY_ATTR(CURRENT_MAX),
	POWER_SUPPLY_ATTR(CURRENT_NOW),
	POWER_SUPPLY_ATTR(CURRENT_AVG),
	POWER_SUPPLY_ATTR(CURRENT_BOOT),
	POWER_SUPPLY_ATTR(POWER_NOW),
	POWER_SUPPLY_ATTR(POWER_AVG),
	POWER_SUPPLY_ATTR(CHARGE_FULL_DESIGN),
	POWER_SUPPLY_ATTR(CHARGE_EMPTY_DESIGN),
	POWER_SUPPLY_ATTR(CHARGE_FULL),
	POWER_SUPPLY_ATTR(CHARGE_EMPTY),
	POWER_SUPPLY_ATTR(CHARGE_NOW),
	POWER_SUPPLY_ATTR(CHARGE_AVG),
	POWER_SUPPLY_ATTR(CHARGE_COUNTER),
	POWER_SUPPLY_ATTR(CONSTANT_CHARGE_CURRENT),
	POWER_SUPPLY_ATTR(CONSTANT_CHARGE_CURRENT_MAX),
	POWER_SUPPLY_ATTR(CONSTANT_CHARGE_VOLTAGE),
	POWER_SUPPLY_ATTR(CONSTANT_CHARGE_VOLTAGE_MAX),
	POWER_SUPPLY_ATTR(CHARGE_CONTROL_LIMIT),
	POWER_SUPPLY_ATTR(CHARGE_CONTROL_LIMIT_MAX),
	POWER_SUPPLY_ATTR(INPUT_CURRENT_LIMIT),
	POWER_SUPPLY_ATTR(ENERGY_FULL_DESIGN),
	POWER_SUPPLY_ATTR(ENERGY_EMPTY_DESIGN),
	POWER_SUPPLY_ATTR(ENERGY_FULL),
	POWER_SUPPLY_ATTR(ENERGY_EMPTY),
	POWER_SUPPLY_ATTR(ENERGY_NOW),
	POWER_SUPPLY_ATTR(ENERGY_AVG),
	POWER_SUPPLY_ATTR(CAPACITY),
	POWER_SUPPLY_ATTR(CAPACITY_ALERT_MIN),
	POWER_SUPPLY_ATTR(CAPACITY_ALERT_MAX),
	POWER_SUPPLY_ATTR(CAPACITY_ERROR_MARGIN),
	POWER_SUPPLY_ENUM_ATTR(CAPACITY_LEVEL),
	POWER_SUPPLY_ATTR(TEMP),
	POWER_SUPPLY_ATTR(TEMP_MAX),
	POWER_SUPPLY_ATTR(TEMP_MIN),
	POWER_SUPPLY_ATTR(TEMP_ALERT_MIN),
	POWER_SUPPLY_ATTR(TEMP_ALERT_MAX),
	POWER_SUPPLY_ATTR(TEMP_AMBIENT),
	POWER_SUPPLY_ATTR(TEMP_AMBIENT_ALERT_MIN),
	POWER_SUPPLY_ATTR(TEMP_AMBIENT_ALERT_MAX),
	POWER_SUPPLY_ATTR(TIME_TO_EMPTY_NOW),
	POWER_SUPPLY_ATTR(TIME_TO_EMPTY_AVG),
	POWER_SUPPLY_ATTR(TIME_TO_FULL_NOW),
	POWER_SUPPLY_ATTR(TIME_TO_FULL_AVG),
	POWER_SUPPLY_ENUM_ATTR(TYPE),
	POWER_SUPPLY_ATTR(USB_TYPE),
	POWER_SUPPLY_ENUM_ATTR(SCOPE),
	POWER_SUPPLY_ATTR(PRECHARGE_CURRENT),
	POWER_SUPPLY_ATTR(CHARGE_TERM_CURRENT),
	POWER_SUPPLY_ATTR(CALIBRATE),
	POWER_SUPPLY_ATTR(OTG_ONLINE),
	POWER_SUPPLY_ATTR(BATTERY_ID),
	POWER_SUPPLY_ATTR(MANUFACTURE_YEAR),
	POWER_SUPPLY_ATTR(MANUFACTURE_MONTH),
	POWER_SUPPLY_ATTR(MANUFACTURE_DAY),
	/* Local extensions */
	POWER_SUPPLY_ATTR(USB_HC),
	POWER_SUPPLY_ATTR(USB_OTG),
	POWER_SUPPLY_ATTR(CHARGE_ENABLED),
	POWER_SUPPLY_ATTR(SET_SHIP_MODE),
	POWER_SUPPLY_ATTR(REAL_TYPE),
	POWER_SUPPLY_ATTR(CHARGE_NOW_RAW),
	POWER_SUPPLY_ATTR(CHARGE_NOW_ERROR),
	POWER_SUPPLY_ATTR(CAPACITY_RAW),
	POWER_SUPPLY_ATTR(BATTERY_CHARGING_ENABLED),
	POWER_SUPPLY_ATTR(CHARGING_ENABLED),
	POWER_SUPPLY_ATTR(STEP_CHARGING_ENABLED),
	POWER_SUPPLY_ATTR(STEP_CHARGING_STEP),
	POWER_SUPPLY_ATTR(PIN_ENABLED),
	POWER_SUPPLY_ATTR(INPUT_SUSPEND),
	POWER_SUPPLY_ATTR(INPUT_VOLTAGE_REGULATION),
	POWER_SUPPLY_ATTR(INPUT_CURRENT_MAX),
	POWER_SUPPLY_ATTR(INPUT_CURRENT_TRIM),
	POWER_SUPPLY_ATTR(INPUT_CURRENT_SETTLED),
	POWER_SUPPLY_ATTR(INPUT_VOLTAGE_SETTLED),
	POWER_SUPPLY_ATTR(VCHG_LOOP_DBC_BYPASS),
	POWER_SUPPLY_ATTR(CHARGE_COUNTER_SHADOW),
	POWER_SUPPLY_ATTR(HI_POWER),
	POWER_SUPPLY_ATTR(LOW_POWER),
	POWER_SUPPLY_ATTR(COOL_TEMP),
	POWER_SUPPLY_ATTR(WARM_TEMP),
	POWER_SUPPLY_ATTR(COLD_TEMP),
	POWER_SUPPLY_ATTR(HOT_TEMP),
	POWER_SUPPLY_ATTR(SYSTEM_TEMP_LEVEL),
	POWER_SUPPLY_ATTR(RESISTANCE),
	POWER_SUPPLY_ATTR(RESISTANCE_CAPACITIVE),
	POWER_SUPPLY_ATTR(RESISTANCE_ID),
	POWER_SUPPLY_ATTR(RESISTANCE_NOW),
	POWER_SUPPLY_ATTR(FLASH_CURRENT_MAX),
	POWER_SUPPLY_ATTR(UPDATE_NOW),
	POWER_SUPPLY_ATTR(ESR_COUNT),
	POWER_SUPPLY_ATTR(BUCK_FREQ),
	POWER_SUPPLY_ATTR(BOOST_CURRENT),
	POWER_SUPPLY_ATTR(SAFETY_TIMER_ENABLE),
	POWER_SUPPLY_ATTR(CHARGE_DONE),
	POWER_SUPPLY_ATTR(FLASH_ACTIVE),
	POWER_SUPPLY_ATTR(FLASH_TRIGGER),
	POWER_SUPPLY_ATTR(FORCE_TLIM),
	POWER_SUPPLY_ATTR(DP_DM),
	POWER_SUPPLY_ATTR(INPUT_CURRENT_LIMITED),
	POWER_SUPPLY_ATTR(INPUT_CURRENT_NOW),
	POWER_SUPPLY_ATTR(CHARGE_QNOVO_ENABLE),
	POWER_SUPPLY_ATTR(CURRENT_QNOVO),
	POWER_SUPPLY_ATTR(VOLTAGE_QNOVO),
	POWER_SUPPLY_ATTR(RERUN_AICL),
	POWER_SUPPLY_ATTR(CYCLE_COUNT_ID),
	POWER_SUPPLY_ATTR(SAFETY_TIMER_EXPIRED),
	POWER_SUPPLY_ATTR(RESTRICTED_CHARGING),
	POWER_SUPPLY_ATTR(CURRENT_CAPABILITY),
	POWER_SUPPLY_ENUM_ATTR(TYPEC_MODE),
	POWER_SUPPLY_ATTR(TYPEC_CC_ORIENTATION),
	POWER_SUPPLY_ENUM_ATTR(TYPEC_POWER_ROLE),
	POWER_SUPPLY_ENUM_ATTR(TYPEC_SRC_RP),
	POWER_SUPPLY_ATTR(PD_ALLOWED),
	POWER_SUPPLY_ATTR(PD_ACTIVE),
	POWER_SUPPLY_ATTR(PD_IN_HARD_RESET),
	POWER_SUPPLY_ATTR(PD_CURRENT_MAX),
	POWER_SUPPLY_ATTR(PD_USB_SUSPEND_SUPPORTED),
	POWER_SUPPLY_ATTR(CHARGER_TEMP),
	POWER_SUPPLY_ATTR(CHARGER_TEMP_MAX),
	POWER_SUPPLY_ATTR(PARALLEL_DISABLE),
	POWER_SUPPLY_ATTR(PE_START),
	POWER_SUPPLY_ATTR(SOC_REPORTING_READY),
	POWER_SUPPLY_ATTR(DEBUG_BATTERY),
	POWER_SUPPLY_ATTR(FCC_DELTA),
	POWER_SUPPLY_ATTR(ICL_REDUCTION),
	POWER_SUPPLY_ATTR(PARALLEL_MODE),
	POWER_SUPPLY_ATTR(DIE_HEALTH),
	POWER_SUPPLY_ATTR(CONNECTOR_HEALTH),
	POWER_SUPPLY_ATTR(CTM_CURRENT_MAX),
	POWER_SUPPLY_ATTR(HW_CURRENT_MAX),
	POWER_SUPPLY_ATTR(PR_SWAP),
	POWER_SUPPLY_ATTR(CC_STEP),
	POWER_SUPPLY_ATTR(CC_STEP_SEL),
	POWER_SUPPLY_ATTR(SW_JEITA_ENABLED),
	POWER_SUPPLY_ATTR(PD_VOLTAGE_MAX),
	POWER_SUPPLY_ATTR(PD_VOLTAGE_MIN),
	POWER_SUPPLY_ATTR(SDP_CURRENT_MAX),
	POWER_SUPPLY_ATTR(FG_RESET_CLOCK),
	POWER_SUPPLY_ATTR(CONNECTOR_TYPE),
	POWER_SUPPLY_ATTR(PARALLEL_BATFET_MODE),
	POWER_SUPPLY_ATTR(PARALLEL_FCC_MAX),
	POWER_SUPPLY_ATTR(MIN_ICL),
	POWER_SUPPLY_ATTR(MOISTURE_DETECTED),
	POWER_SUPPLY_ATTR(BATT_PROFILE_VERSION),
	POWER_SUPPLY_ATTR(BATT_FULL_CURRENT),
	POWER_SUPPLY_ATTR(RECHARGE_SOC),
	POWER_SUPPLY_ATTR(HVDCP_OPTI_ALLOWED),
	POWER_SUPPLY_ATTR(SMB_EN_MODE),
	POWER_SUPPLY_ATTR(SMB_EN_REASON),
	POWER_SUPPLY_ATTR(ESR_ACTUAL),
	POWER_SUPPLY_ATTR(ESR_NOMINAL),
	POWER_SUPPLY_ATTR(SOH),
	POWER_SUPPLY_ATTR(CLEAR_SOH),
	POWER_SUPPLY_ATTR(FORCE_RECHARGE),
	POWER_SUPPLY_ATTR(FCC_STEPPER_ENABLE),
	POWER_SUPPLY_ATTR(TOGGLE_STAT),
	POWER_SUPPLY_ATTR(MAIN_FCC_MAX),
	POWER_SUPPLY_ATTR(FG_RESET),
	POWER_SUPPLY_ATTR(QC_OPTI_DISABLE),
	POWER_SUPPLY_ATTR(CC_SOC),
	POWER_SUPPLY_ATTR(BATT_AGE_LEVEL),
	POWER_SUPPLY_ATTR(SCALE_MODE_EN),
	POWER_SUPPLY_ATTR(VOLTAGE_VPH),
	POWER_SUPPLY_ATTR(CHIP_VERSION),
	POWER_SUPPLY_ATTR(THERM_ICL_LIMIT),
	POWER_SUPPLY_ATTR(DC_RESET),
	POWER_SUPPLY_ATTR(VOLTAGE_MAX_LIMIT),
	POWER_SUPPLY_ATTR(REAL_CAPACITY),
	POWER_SUPPLY_ATTR(FORCE_MAIN_ICL),
	POWER_SUPPLY_ATTR(FORCE_MAIN_FCC),
	POWER_SUPPLY_ATTR(COMP_CLAMP_LEVEL),
	POWER_SUPPLY_ATTR(ADAPTER_CC_MODE),
	POWER_SUPPLY_ATTR(SKIN_HEALTH),
	POWER_SUPPLY_ATTR(AICL_DONE),
	POWER_SUPPLY_ATTR(VOLTAGE_STEP),
	POWER_SUPPLY_ATTR(APSD_RERUN),
	POWER_SUPPLY_ATTR(APSD_TIMEOUT),
	/* Charge pump properties */
	POWER_SUPPLY_ATTR(CP_STATUS1),
	POWER_SUPPLY_ATTR(CP_STATUS2),
	POWER_SUPPLY_ATTR(CP_ENABLE),
	POWER_SUPPLY_ATTR(CP_SWITCHER_EN),
	POWER_SUPPLY_ATTR(CP_DIE_TEMP),
	POWER_SUPPLY_ATTR(CP_ISNS),
	POWER_SUPPLY_ATTR(CP_ISNS_SLAVE),
	POWER_SUPPLY_ATTR(CP_TOGGLE_SWITCHER),
	POWER_SUPPLY_ATTR(CP_IRQ_STATUS),
	POWER_SUPPLY_ATTR(CP_ILIM),
	POWER_SUPPLY_ATTR(IRQ_STATUS),
	POWER_SUPPLY_ATTR(PARALLEL_OUTPUT_MODE),
	POWER_SUPPLY_ATTR(CC_TOGGLE_ENABLE),
	POWER_SUPPLY_ATTR(FG_TYPE),
	POWER_SUPPLY_ATTR(CHARGER_STATUS),
	/* Local extensions of type int64_t */
	POWER_SUPPLY_ATTR(CHARGE_COUNTER_EXT),
	/* Properties of type `const char *' */
	POWER_SUPPLY_ATTR(MODEL_NAME),
	POWER_SUPPLY_ATTR(MANUFACTURER),
	POWER_SUPPLY_ATTR(BATTERY_TYPE),
	POWER_SUPPLY_ATTR(CYCLE_COUNTS),
	POWER_SUPPLY_ATTR(SERIAL_NUMBER),
	POWER_SUPPLY_ATTR(QUICK_CHARGE_TYPE),
};

static struct attribute *
__power_supply_attrs[ARRAY_SIZE(power_supply_attrs) + 1];

static struct power_supply_attr *to_ps_attr(struct device_attribute *attr)
{
	return container_of(attr, struct power_supply_attr, dev_attr);
}

static enum power_supply_property dev_attr_psp(struct device_attribute *attr)
{
	return  to_ps_attr(attr) - power_supply_attrs;
}

static ssize_t power_supply_show_usb_type(struct device *dev,
					  const struct power_supply_desc *desc,
					  union power_supply_propval *value,
					  char *buf)
{
	enum power_supply_usb_type usb_type;
	ssize_t count = 0;
	bool match = false;
	int i;

	for (i = 0; i < desc->num_usb_types; ++i) {
		usb_type = desc->usb_types[i];

		if (value->intval == usb_type) {
			count += sprintf(buf + count, "[%s] ",
					 POWER_SUPPLY_USB_TYPE_TEXT[usb_type]);
			match = true;
		} else {
			count += sprintf(buf + count, "%s ",
					 POWER_SUPPLY_USB_TYPE_TEXT[usb_type]);
		}
	}

	if (!match) {
		dev_warn(dev, "driver reporting unsupported connected type\n");
		return -EINVAL;
	}

	if (count)
		buf[count - 1] = '\n';

	return count;
}

static ssize_t power_supply_show_property(struct device *dev,
					  struct device_attribute *attr,
					  char *buf) {
	ssize_t ret;
	struct power_supply *psy = dev_get_drvdata(dev);
	struct power_supply_attr *ps_attr = to_ps_attr(attr);
	enum power_supply_property psp = dev_attr_psp(attr);
	union power_supply_propval value;

	if (psp == POWER_SUPPLY_PROP_TYPE) {
		value.intval = psy->desc->type;
	} else {
		ret = power_supply_get_property(psy, psp, &value);

		if (ret < 0) {
			if (ret == -ENODATA)
				dev_dbg_ratelimited(dev,
					"driver has no data for `%s' property\n",
					attr->attr.name);
			else if (ret != -ENODEV && ret != -EAGAIN)
				dev_err_ratelimited(dev,
					"driver failed to report `%s' property: %zd\n",
					attr->attr.name, ret);
			return ret;
		}
	}

	if (ps_attr->text_values_len > 0 &&
	    value.intval < ps_attr->text_values_len && value.intval >= 0) {
		return sprintf(buf, "%s\n", ps_attr->text_values[value.intval]);
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_USB_TYPE:
		ret = power_supply_show_usb_type(dev, psy->desc,
						&value, buf);
		break;
	case POWER_SUPPLY_PROP_DIE_HEALTH:
	case POWER_SUPPLY_PROP_SKIN_HEALTH:
	case POWER_SUPPLY_PROP_CHARGE_COUNTER_EXT:
		ret = sprintf(buf, "%lld\n", value.int64val);
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME ... POWER_SUPPLY_PROP_SERIAL_NUMBER:
		ret = sprintf(buf, "%s\n", value.strval);
		break;
	default:
		ret = sprintf(buf, "%d\n", value.intval);
	}

	return ret;
}

static ssize_t power_supply_store_property(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count) {
	ssize_t ret;
	struct power_supply *psy = dev_get_drvdata(dev);
	struct power_supply_attr *ps_attr = to_ps_attr(attr);
	enum power_supply_property psp = dev_attr_psp(attr);
	union power_supply_propval value;

	ret = -EINVAL;
	if (ps_attr->text_values_len > 0) {
		ret = __sysfs_match_string(ps_attr->text_values,
					   ps_attr->text_values_len, buf);
	}

	/*
	 * If no match was found, then check to see if it is an integer.
	 * Integer values are valid for enums in addition to the text value.
	 */
	if (ret < 0) {
		long long_val;

		ret = kstrtol(buf, 10, &long_val);
		if (ret < 0)
			return ret;

		ret = long_val;
	}

	value.intval = ret;

	ret = power_supply_set_property(psy, psp, &value);
	if (ret < 0)
		return ret;

	return count;
}

static umode_t power_supply_attr_is_visible(struct kobject *kobj,
					   struct attribute *attr,
					   int attrno)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct power_supply *psy = dev_get_drvdata(dev);
	umode_t mode = S_IRUSR | S_IRGRP | S_IROTH;
	int i;

	if (!power_supply_attrs[attrno].prop_name)
		return 0;

	if (attrno == POWER_SUPPLY_PROP_TYPE)
		return mode;

	for (i = 0; i < psy->desc->num_properties; i++) {
		int property = psy->desc->properties[i];

		if (property == attrno) {
			if (psy->desc->property_is_writeable &&
			    psy->desc->property_is_writeable(psy, property) > 0)
				mode |= S_IWUSR;

			return mode;
		}
	}

	return 0;
}

static struct attribute_group power_supply_attr_group = {
	.attrs = __power_supply_attrs,
	.is_visible = power_supply_attr_is_visible,
};

static const struct attribute_group *power_supply_attr_groups[] = {
	&power_supply_attr_group,
	NULL,
};

static void str_to_lower(char *str)
{
	while (*str) {
		*str = tolower(*str);
		str++;
	}
}

void power_supply_init_attrs(struct device_type *dev_type)
{
	int i;

	dev_type->groups = power_supply_attr_groups;

	for (i = 0; i < ARRAY_SIZE(power_supply_attrs); i++) {
		struct device_attribute *attr;

		if (!power_supply_attrs[i].prop_name) {
			pr_warn("%s: Property %d skipped because is is missing from power_supply_attrs\n",
				__func__, i);
			sprintf(power_supply_attrs[i].attr_name, "_err_%d", i);
		} else {
			str_to_lower(power_supply_attrs[i].attr_name);
		}

		attr = &power_supply_attrs[i].dev_attr;

		attr->attr.name = power_supply_attrs[i].attr_name;
		attr->show = power_supply_show_property;
		attr->store = power_supply_store_property;
		__power_supply_attrs[i] = &attr->attr;
	}
}

static int add_prop_uevent(struct device *dev, struct kobj_uevent_env *env,
			   enum power_supply_property prop, char *prop_buf)
{
	int ret = 0;
	struct power_supply_attr *pwr_attr;
	struct device_attribute *dev_attr;
	char *line;

	pwr_attr = &power_supply_attrs[prop];
	dev_attr = &pwr_attr->dev_attr;

	ret = power_supply_show_property(dev, dev_attr, prop_buf);
	if (ret == -ENODEV || ret == -ENODATA) {
		/*
		 * When a battery is absent, we expect -ENODEV. Don't abort;
		 * send the uevent with at least the the PRESENT=0 property
		 */
		return 0;
	}

	if (ret < 0)
		return ret;

	line = strchr(prop_buf, '\n');
	if (line)
		*line = 0;

	return add_uevent_var(env, "POWER_SUPPLY_%s=%s",
			      pwr_attr->prop_name, prop_buf);
}

int power_supply_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	int ret = 0, j;
	char *prop_buf;

	if (!psy || !psy->desc) {
		dev_dbg(dev, "No power supply yet\n");
		return ret;
	}

	ret = add_uevent_var(env, "POWER_SUPPLY_NAME=%s", psy->desc->name);
	if (ret)
		return ret;

	prop_buf = (char *)get_zeroed_page(GFP_KERNEL);
	if (!prop_buf)
		return -ENOMEM;

	ret = add_prop_uevent(dev, env, POWER_SUPPLY_PROP_TYPE, prop_buf);
	if (ret)
		goto out;

	for (j = 0; j < psy->desc->num_properties; j++) {
		ret = add_prop_uevent(dev, env, psy->desc->properties[j],
				      prop_buf);
		if (ret)
			goto out;
	}

out:
	free_page((unsigned long)prop_buf);

	return ret;
}
