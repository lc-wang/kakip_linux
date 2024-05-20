// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas RSCI I2C driver
 *
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

enum riic_chip_id{
	RSCI_V2H_REGTYPE,
	RSCI_NR_REGTYPES,
};

enum {
	RSCI_RDR,					/* Receive Data Register */
	RSCI_TDR,					/* Transmit Data Register */
	RSCI_CCR0,					/* Common Control Register 0 */
	RSCI_CCR1,					/* Common Control Register 1 */
	RSCI_CCR2,					/* Common Control Register 2 */
	RSCI_CCR3,					/* Common Control Register 3 */
	RSCI_CCR4,					/* Common Control Register 4 */
	RSCI_ICR,					/* Simple I2C Control Register */
	RSCI_FCR,					/* FIFO Control Register */
	RSCI_DCR,					/* Driver Control Register */
	RSCI_CSR,					/* Common Status Register */
	RSCI_ISR,					/* Simple I2C Status Register */
	RSCI_FRSR,					/* FIFO Receive Status Register */
	RSCI_FTSR,					/* FIFO Transmit Status Register */
	RSCI_CFCLR,					/* Common Flag CLear Register */
	RSCI_ICFCLR,					/* Simple I2C Flag Clear Register */
	RSCI_FFCLR,					/* FIFO Flag CLear Register */

	RSCI_NR_REG
};

struct riic_port_params {
		const int regs[RSCI_NR_REG];
};

static const struct riic_port_params riic_port_params[RSCI_NR_REGTYPES] = {

	[RSCI_V2H_REGTYPE] = {
	    .regs = {
		[RSCI_RDR]	=  0x00,
		[RSCI_TDR]	=  0x04,
		[RSCI_CCR0]	=  0x08,
		[RSCI_CCR1]	=  0x0C,
		[RSCI_CCR2]	=  0x10,
		[RSCI_CCR3]	=  0x14,
		[RSCI_CCR4]	=  0x18,
		[RSCI_ICR]	=  0x20,
		[RSCI_FCR]	=  0x24,
		[RSCI_DCR]	=  0x30,
		[RSCI_CSR]	=  0x48,
		[RSCI_ISR]	=  0x4C,
		[RSCI_FRSR]	=  0x50,
		[RSCI_FTSR]	=  0x54,
		[RSCI_CFCLR]	=  0x68,
		[RSCI_ICFCLR]	=  0x6C,
		[RSCI_FFCLR]	=  0x70,
	    },
	},
};


#define RSCI_RDR_MASK		GENMASK(7, 0)
#define RSCI_TDR_MASK		GENMASK(7, 0)

#define CCR0_RE			BIT(0)
#define CCR0_TE			BIT(4)
#define CCR0_RIE		BIT(16)
#define CCR0_TIE		BIT(20)
#define CCR0_TEIE		BIT(21)

#define CCR1_NFCS(_x)		((_x) << 24)
#define CCR1_NFEN		BIT(28)

#define CCR2_BCP		0x00000005
#define CCR2_BRR(_x)		((_x) << 8)
#define CCR2_BRME		BIT(16)
#define CCR2_CKS(_x)		((_x) << 20)
#define CCR2_MDDR(_x)		((_x) << 24)	/* Should be 1 on writes */

#define CCR3_CHR(_x)		((_x) << 8)
#define CCR3_LSBF		BIT(12)
#define CCR3_MOD		(4 << 16)

#define ICR_IICDL		(2 << 0)
#define ICR_IICINTM		BIT(8)
#define ICR_IICCSC		BIT(9)
#define ICR_IICACKT(_x)		((_x) << 13)
#define ICR_IICSTAREQ		BIT(16)
#define ICR_IICRSTAREQ		BIT(17)
#define ICR_IICSTPREQ		BIT(18)
#define ICR_IICSDAS(_x)		((_x) << 20)
#define ICR_IICSDAS_MASK	GENMASK(21, 20)
#define ICR_IICSCLS(_x)		((_x) << 22)
#define ICR_IICSCLS_MASK	GENMASK(23, 22)

#define ISR_IICACKR		BIT(0)

#define CFCLR_ERSC		BIT(4)
#define CFCLR_DCMFC		BIT(16)
#define CFCLR_DPERC		BIT(17)
#define CFCLR_DFERC		BIT(18)
#define CFCLR_ORERC		BIT(24)
#define CFCLR_MFFC		BIT(26)
#define CFCLR_PERC		BIT(27)
#define CFCLR_FERC		BIT(28)
#define CFCLR_RDRFC		BIT(31)

#define ICFCLR_IICSTIFC		BIT(3)

#define RIIC_INIT_MSG	-1

struct rsci_dev {
	void __iomem *base;
	u8 *buf;
	struct i2c_msg *msg;
	int bytes_left;
	int err;
	int is_last;
	struct completion msg_done;
	struct i2c_adapter adapter;
	struct clk *clk;
	enum riic_chip_id chip_id;
};

struct rsci_irq_desc {
	int res_num;
	irq_handler_t isr;
	char *name;
};

static inline u32 rz_rsci_read_reg(struct rsci_dev *riic, int offset)
{
	return readl(riic->base + riic_port_params[riic->chip_id].regs[offset]);
}

static inline void rz_rsci_write_reg(u32 val,
				 struct rsci_dev *riic, int offset)
{
	writel(val, riic->base + riic_port_params[riic->chip_id].regs[offset]);
}

static inline void rz_rsci_clear_set_bit(struct rsci_dev *riic,
				u32 clear, u32 set, u8 reg)
{
	writel((readl(riic->base + riic_port_params[riic->chip_id].regs[reg])
			& ~clear) | set, riic->base +
			riic_port_params[riic->chip_id].regs[reg]);
}

static int rsci_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	struct rsci_dev *riic = i2c_get_adapdata(adap);
	unsigned long time_left;
	int i;
	u32 start_bit;
	u32 icr, ccr0;

	pm_runtime_get_sync(adap->dev.parent);
	reinit_completion(&riic->msg_done);
	riic->err = 0;
	for (i = 0, start_bit = ICR_IICSTAREQ; i < num; i++) {
		riic->bytes_left = RIIC_INIT_MSG;
		riic->buf = msgs[i].buf;
		riic->msg = &msgs[i];
		riic->is_last = (i == num - 1);

		/* Enable TE, RE, TXI and TEIE. */
		ccr0 = CCR0_RE | CCR0_TE | CCR0_TIE | CCR0_TEIE;
		rz_rsci_write_reg(ccr0, riic, RSCI_CCR0);

		/* Initiate a start condition.
		* - The IICSTARREQ, IICSDAS, IICSCLS bits must be set simultaneously.
		* - IICDL, IICINTM, IICCSC, and IICACKT settings must be preserved.
		*/
		icr = ICR_IICSDAS(1) | ICR_IICSCLS(1) | start_bit;
		rz_rsci_clear_set_bit(riic, ICR_IICSDAS_MASK | ICR_IICSCLS_MASK | start_bit, icr, RSCI_ICR);

		time_left = wait_for_completion_timeout(&riic->msg_done, riic->adapter.timeout);
		if (time_left == 0)
			riic->err = -ETIMEDOUT;

		if (riic->err)
			break;

		start_bit = ICR_IICRSTAREQ;
	}

	pm_runtime_put(adap->dev.parent);

	return riic->err ?: num;
}

static irqreturn_t rsci_tdre_isr(int irq, void *data)
{
	struct rsci_dev *riic = data;
	u8 val;
	u32 icr;

	if (!riic->bytes_left) {
		icr = ICR_IICSDAS(1) | ICR_IICSCLS(1) | ICR_IICSTPREQ;
		rz_rsci_clear_set_bit(riic, ICR_IICSDAS_MASK | ICR_IICSCLS_MASK | ICR_IICSTPREQ, icr, RSCI_ICR);

		return IRQ_HANDLED;
	}

	if ((rz_rsci_read_reg(riic, RSCI_ISR) & ISR_IICACKR)) {
		riic->err = -ENXIO;
		riic->bytes_left = 0;
		icr = ICR_IICSDAS(1) | ICR_IICSCLS(1) | ICR_IICSTPREQ;
		rz_rsci_clear_set_bit(riic, ICR_IICSDAS_MASK | ICR_IICSCLS_MASK | ICR_IICSTPREQ, icr, RSCI_ICR);
		return IRQ_HANDLED;
	}

	if (riic->bytes_left == RIIC_INIT_MSG) {
		if (riic->msg->flags & I2C_M_RD) {
			rz_rsci_clear_set_bit(riic, ICR_IICACKT(1), ICR_IICACKT(0), RSCI_ICR);
			rz_rsci_clear_set_bit(riic, CCR0_RIE, CCR0_RIE, RSCI_CCR0);

			riic->bytes_left = riic->msg->len;
			if (riic->bytes_left == 1) {
				rz_rsci_clear_set_bit(riic, ICR_IICACKT(1), ICR_IICACKT(1), RSCI_ICR);
				rz_rsci_clear_set_bit(riic, RSCI_TDR_MASK, RSCI_TDR_MASK, RSCI_TDR);
			} else {
				rz_rsci_clear_set_bit(riic, RSCI_TDR_MASK, RSCI_TDR_MASK, RSCI_TDR);
			}
		} else {
			riic->bytes_left = riic->msg->len;
			if (!riic->bytes_left) {
				icr = ICR_IICSDAS(1) | ICR_IICSCLS(1) | ICR_IICSTPREQ;
				rz_rsci_clear_set_bit(riic, ICR_IICSDAS_MASK | ICR_IICSCLS_MASK | ICR_IICSTPREQ, icr, RSCI_ICR);
				return IRQ_HANDLED;
			}

			val = *riic->buf;
			riic->buf++;
			riic->bytes_left--;
			rz_rsci_clear_set_bit(riic, RSCI_TDR_MASK, val, RSCI_TDR);
		}
	} else {
		if (riic->msg->flags & I2C_M_RD) {
			rz_rsci_clear_set_bit(riic, ICR_IICACKT(1), ICR_IICACKT(0), RSCI_ICR);
			rz_rsci_clear_set_bit(riic, CCR0_RIE, CCR0_RIE, RSCI_CCR0);
			if (riic->bytes_left == 1) {
				rz_rsci_clear_set_bit(riic, ICR_IICACKT(1), ICR_IICACKT(1), RSCI_ICR);
				rz_rsci_clear_set_bit(riic, RSCI_TDR_MASK, RSCI_TDR_MASK, RSCI_TDR);
			} else {
				rz_rsci_clear_set_bit(riic, RSCI_TDR_MASK, RSCI_TDR_MASK, RSCI_TDR);
			}
		} else {
			val = *riic->buf;
			riic->buf++;
			riic->bytes_left--;
			rz_rsci_clear_set_bit(riic, RSCI_TDR_MASK, val, RSCI_TDR);
		}
	}

	return IRQ_HANDLED;
}

static irqreturn_t rsci_tend_isr(int irq, void *data)
{
	struct rsci_dev *riic = data;
	u8 val;
	u32 icr;

	if (!riic->bytes_left) {
		icr = ICR_IICSDAS(3) | ICR_IICSCLS(3);
		rz_rsci_clear_set_bit(riic, ICR_IICSDAS_MASK | ICR_IICSCLS_MASK, icr, RSCI_ICR);
		rz_rsci_write_reg(ICFCLR_IICSTIFC, riic, RSCI_ICFCLR);
		complete(&riic->msg_done);
		return IRQ_HANDLED;
	}



	if (riic->bytes_left == RIIC_INIT_MSG) {
		val = i2c_8bit_addr_from_msg(riic->msg);
		rz_rsci_write_reg(ICFCLR_IICSTIFC, riic, RSCI_ICFCLR);
		icr = ICR_IICSDAS(0) | ICR_IICSCLS(0);
		rz_rsci_clear_set_bit(riic, ICR_IICSDAS_MASK | ICR_IICSCLS_MASK, icr, RSCI_ICR);

		rz_rsci_clear_set_bit(riic, RSCI_TDR_MASK, val, RSCI_TDR);
	}

	return IRQ_HANDLED;
}

static irqreturn_t rsci_rdrf_isr(int irq, void *data)
{
	struct rsci_dev *riic = data;

	*riic->buf = rz_rsci_read_reg(riic, RSCI_RDR) & RSCI_RDR_MASK;
	riic->buf++;
	riic->bytes_left--;

	return IRQ_HANDLED;
}

static irqreturn_t rsci_error_isr(int irq, void *data)
{
	return IRQ_HANDLED;
}

static u32 rsci_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm riic_algo = {
	.master_xfer	= rsci_i2c_xfer,
	.functionality	= rsci_i2c_func,
};

static int rsci_i2c_init_hw(struct rsci_dev *riic, struct i2c_timings *t)
{
	int ret = 0;
	unsigned long rate;
	unsigned int cks = 0, c, br, scrate, prediv, sr = 16;
	int brr = 30;
	u32 ccr2, icr, cfclr;

	clk_prepare_enable(riic->clk);
	pm_runtime_get_sync(riic->adapter.dev.parent);

	rz_rsci_write_reg(0, riic, RSCI_CCR0);
	icr = ICR_IICSDAS(3) | ICR_IICSCLS(3) | ICR_IICDL | ICR_IICINTM | ICR_IICACKT(1) | ICR_IICCSC;
	rz_rsci_write_reg(icr, riic, RSCI_ICR);

	rz_rsci_write_reg(CCR3_MOD & (~CCR3_LSBF), riic, RSCI_CCR3);

	if (t->bus_freq_hz > I2C_MAX_FAST_MODE_FREQ) {
		dev_err(&riic->adapter.dev,
			"unsupported bus speed (%dHz). %d max\n",
			t->bus_freq_hz, I2C_MAX_FAST_MODE_FREQ);
		ret = -EINVAL;
		goto out;
	}

	rate = clk_get_rate(riic->clk);

	for (c = 0; c <= 3; c++) {
		prediv = sr << (2 * c + 1);

		if (t->bus_freq_hz > UINT_MAX / prediv)
			break;

		scrate = prediv * t->bus_freq_hz;

		br = DIV_ROUND_CLOSEST(rate, scrate);
		br = clamp(br, 1U, 256U);

		brr = br - 1;
		cks = c;
		if (brr <= 255 && brr >= 0)
			break;
	}

	ccr2 = (CCR2_MDDR(0xFF) | CCR2_CKS(cks) | CCR2_BRR(brr) | CCR2_BCP) & ~CCR2_BRME;
	rz_rsci_write_reg(ccr2, riic, RSCI_CCR2);
	rz_rsci_write_reg(CCR1_NFCS(1) | CCR1_NFEN, riic, RSCI_CCR1);

	cfclr = CFCLR_RDRFC | CFCLR_FERC | CFCLR_PERC | CFCLR_MFFC | CFCLR_ORERC
		| CFCLR_DFERC | CFCLR_DPERC | CFCLR_DCMFC | CFCLR_ERSC;

	rz_rsci_write_reg(cfclr, riic, RSCI_CFCLR);
	rz_rsci_write_reg(ICFCLR_IICSTIFC, riic, RSCI_ICFCLR);
	rz_rsci_write_reg(CCR0_RE | CCR0_TE, riic, RSCI_CCR0);


	pr_debug("rsci-i2c: freq=%u, , cks=%d, brr=%d \n",
		t->bus_freq_hz , cks, brr);
out:
	pm_runtime_put(riic->adapter.dev.parent);
	return ret;
}

static struct rsci_irq_desc rsci_irqs[] = {
	{ .res_num = 0, .isr = rsci_error_isr, .name = "rz-rsci-error" },
	{ .res_num = 1, .isr = rsci_rdrf_isr,  .name = "rz-rsci-rdrf" },
	{ .res_num = 2, .isr = rsci_tdre_isr,  .name = "rz-rsci-tdre" },
	{ .res_num = 3, .isr = rsci_tend_isr,  .name = "rz-rsci-tend" },
};

static void rsci_reset_control_assert(void *data)
{
	reset_control_assert(data);
}

static int rz_rsci_i2c_probe(struct platform_device *pdev)
{
	struct rsci_dev *riic;
	struct i2c_adapter *adap;
	struct i2c_timings i2c_t;
	struct reset_control *rstc;
	int i, ret;
	enum riic_chip_id chip_id;

	chip_id = (uintptr_t)of_device_get_match_data(&pdev->dev);

	riic = devm_kzalloc(&pdev->dev, sizeof(*riic), GFP_KERNEL);
	if (!riic)
		return -ENOMEM;

	riic->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(riic->base))
		return PTR_ERR(riic->base);

	riic->clk = devm_clk_get(&pdev->dev, "fck");
	if (IS_ERR(riic->clk)) {
		dev_err(&pdev->dev, "missing controller clock");
		return PTR_ERR(riic->clk);
	}

	rstc = devm_reset_control_get_optional_exclusive(&pdev->dev, NULL);
	if (IS_ERR(rstc))
		return dev_err_probe(&pdev->dev, PTR_ERR(rstc),
				     "Error: missing reset ctrl\n");

	ret = reset_control_deassert(rstc);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&pdev->dev, rsci_reset_control_assert, rstc);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(rsci_irqs); i++) {
		ret = platform_get_irq(pdev, rsci_irqs[i].res_num);
		if (ret < 0)
			return ret;

		ret = devm_request_irq(&pdev->dev, ret, rsci_irqs[i].isr,
				       0, rsci_irqs[i].name, riic);
		if (ret) {
			dev_err(&pdev->dev, "failed to request irq %s\n", rsci_irqs[i].name);
			return ret;
		}
	}

	riic->chip_id = chip_id;
	adap = &riic->adapter;
	i2c_set_adapdata(adap, riic);
	strscpy(adap->name, "Renesas RSCI I2C adapter", sizeof(adap->name));
	adap->owner = THIS_MODULE;
	adap->algo = &riic_algo;
	adap->dev.parent = &pdev->dev;
	adap->dev.of_node = pdev->dev.of_node;

	init_completion(&riic->msg_done);

	i2c_parse_fw_timings(&pdev->dev, &i2c_t, true);

	pm_runtime_enable(&pdev->dev);

	ret = rsci_i2c_init_hw(riic, &i2c_t);
	if (ret)
		goto out;

	ret = i2c_add_adapter(adap);
	if (ret)
		goto out;

	platform_set_drvdata(pdev, riic);

	dev_info(&pdev->dev, "registered with %dHz bus speed\n",
		 i2c_t.bus_freq_hz);
	return 0;

out:
	pm_runtime_disable(&pdev->dev);
	return ret;
}

static int rz_rsci_i2c_remove(struct platform_device *pdev)
{
	struct rsci_dev *riic = platform_get_drvdata(pdev);

	pm_runtime_get_sync(&pdev->dev);
	rz_rsci_write_reg(0, riic, RSCI_CCR0);
	pm_runtime_put(&pdev->dev);
	i2c_del_adapter(&riic->adapter);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static const struct of_device_id rz_rsci_i2c_dt_ids[] = {
	{ .compatible = "renesas,r9a09g057-rz-rsci-i2c", .data = (void *)RSCI_V2H_REGTYPE },
	{ /* Sentinel */ },
};

static struct platform_driver rz_rsci_i2c_driver = {
	.probe		= rz_rsci_i2c_probe,
	.remove		= rz_rsci_i2c_remove,
	.driver		= {
		.name	= "rz-rsci-i2c",
		.of_match_table = rz_rsci_i2c_dt_ids,
	},
};

module_platform_driver(rz_rsci_i2c_driver);

MODULE_DESCRIPTION("Renesas RSCI I2C adapter");
MODULE_AUTHOR("Tranh Ha <tranh.ha.xb@renesas.com>");
MODULE_LICENSE("GPL v2");
MODULE_DEVICE_TABLE(of, rz_sci_i2c_dt_ids);
