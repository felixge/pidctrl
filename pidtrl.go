package pidctrl

import (
	"time"
)

func NewPIDController(p, i, d float64) *PIDController {
	return &PIDController{p: p, i: i, d: d}
}

type PIDController struct {
	p          float64
	i          float64
	d          float64
	setpoint   float64
	prevErr    float64
	integral   float64
	lastUpdate time.Time
}

func (c *PIDController) Set(setpoint float64) {
	c.setpoint = setpoint
}

func (c *PIDController) UpdateDuration(value float64, duration time.Duration) float64 {
	var (
		dt = duration.Seconds()
		err = c.setpoint - value
		d float64
	)
	c.integral += err * dt
	if dt > 0 {
		d = (err - c.prevErr) / dt
	}
	c.prevErr = err
	return (c.p * err) + (c.i * c.integral) + (c.d * d)
}
