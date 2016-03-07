package pidctrl

import (
	"fmt"
	"reflect"
	"testing"
	"time"
)

var tests = []struct {
	p       float64
	i       float64
	d       float64
	min     float64
	max     float64
	updates []*testUpdate
}{
	// p-only controller
	{
		p: 0.5,
		updates: []*testUpdate{
			{setpoint: 10, input: 5, output: 2.5},
			{input: 10, output: 0},
			{input: 15, output: -2.5},
			{input: 100, output: -45},
			{setpoint: 1, input: 0, output: 0.5},
		},
	},
	// i-only controller
	{
		i: 0.5,
		updates: []*testUpdate{
			{setpoint: 10, input: 5, duration: time.Second, output: 2.5},
			{input: 5, duration: time.Second, output: 5},
			{input: 5, duration: time.Second, output: 7.5},
			{input: 15, duration: time.Second, output: 5},
			{input: 20, duration: time.Second, output: 0},
		},
	},
	// d-only controller
	{
		d: 0.5,
		updates: []*testUpdate{
			{setpoint: 10, input: 5, duration: time.Second, output: -2.5},
			{input: 5, duration: time.Second, output: 0},
			{input: 10, duration: time.Second, output: -2.5},
		},
	},
	// pid controller
	{
		p: 0.5,
		i: 0.5,
		d: 0.5,
		updates: []*testUpdate{
			{setpoint: 10, input: 5, duration: time.Second, output: 2.5},
			{input: 10, duration: time.Second, output: 0},
			{input: 15, duration: time.Second, output: -5},
			{input: 100, duration: time.Second, output: -132.5},
			{setpoint: 1, duration: time.Second, input: 0, output: 6},
		},
	},
	// Thermostat example
	{
		p:   0.6,
		i:   1.2,
		d:   0.075,
		max: 1, // on or off
		updates: []*testUpdate{
			{setpoint: 72, input: 50, duration: time.Second, output: 1},
			{input: 51, duration: time.Second, output: 1},
			{input: 55, duration: time.Second, output: 1},
			{input: 60, duration: time.Second, output: 1},
			{input: 75, duration: time.Second, output: 0},
			{input: 76, duration: time.Second, output: 0},
			{input: 74, duration: time.Second, output: 0},
			{input: 72, duration: time.Second, output: 0.15},
			{input: 71, duration: time.Second, output: 1},
		},
	},
	// pd controller, demonstrating we need to prevent i windup before summing up output
	{
		p:   40.0,
		i:   0,
		d:   12.0,
		max: 255, min: 0,
		updates: []*testUpdate{
			{setpoint: 90.0, input: 22.00, duration: time.Second, output: 255.00},
			{input: 25.29, duration: time.Second, output: 255.00},
			{input: 28.56, duration: time.Second, output: 255.00},
			{input: 31.80, duration: time.Second, output: 255.00},
			{input: 35.02, duration: time.Second, output: 255.00},
			{input: 38.21, duration: time.Second, output: 255.00},
			{input: 41.38, duration: time.Second, output: 255.00},
			{input: 44.53, duration: time.Second, output: 255.00},
			{input: 47.66, duration: time.Second, output: 255.00},
			{input: 50.76, duration: time.Second, output: 255.00},
			{input: 53.84, duration: time.Second, output: 255.00},
			{input: 56.90, duration: time.Second, output: 255.00},
			{input: 59.93, duration: time.Second, output: 255.00},
			{input: 62.95, duration: time.Second, output: 255.00},
			{input: 65.94, duration: time.Second, output: 255.00},
			{input: 68.91, duration: time.Second, output: 255.00},
			{input: 71.85, duration: time.Second, output: 255.00},
			{input: 74.78, duration: time.Second, output: 255.00},
			{input: 77.69, duration: time.Second, output: 255.00},
			{input: 80.57, duration: time.Second, output: 255.00},
			{input: 83.43, duration: time.Second, output: 228.48},
			{input: 85.93, duration: time.Second, output: 132.80},
			{input: 87.18, duration: time.Second, output: 97.80},
			{input: 87.96, duration: time.Second, output: 72.24},
			{input: 88.41, duration: time.Second, output: 58.20},
			{input: 88.68, duration: time.Second, output: 49.56},
			{input: 88.83, duration: time.Second, output: 45.00},
			{input: 88.92, duration: time.Second, output: 42.12},
			{input: 88.98, duration: time.Second, output: 40.08},
			{input: 89.00, duration: time.Second, output: 39.76},
			{input: 89.03, duration: time.Second, output: 38.44},
			{input: 89.03, duration: time.Second, output: 38.80},
			{input: 89.05, duration: time.Second, output: 37.76},
			{input: 89.04, duration: time.Second, output: 38.52},
			{input: 89.05, duration: time.Second, output: 37.88},
			{input: 89.05, duration: time.Second, output: 38.00},
			{input: 89.05, duration: time.Second, output: 38.00},
			{input: 89.05, duration: time.Second, output: 38.00},
			{input: 89.05, duration: time.Second, output: 38.00},
		},
	},
	// panic test
	{
		p:       0.5,
		i:       0.5,
		d:       0.5,
		min:     100, // min and max are swapped
		max:     1,
		updates: []*testUpdate{},
	},
}

type testUpdate struct {
	setpoint float64
	input    float64
	duration time.Duration
	output   float64
}

func round(v float64, decimals int) float64 {
	var pow float64 = 1
	for i := 0; i < decimals; i++ {
		pow *= 10
	}
	return float64(int((v*pow)+0.5)) / pow
}

func (u *testUpdate) check(c *PIDController) error {
	if u.setpoint != 0 {
		c.Set(u.setpoint)
	}
	output := c.UpdateDuration(u.input, u.duration)
	if round(output, 63) != round(u.output, 63) {
		return fmt.Errorf("Bad output: %f != %f (%#v)", output, u.output, u)
	}
	return nil
}

func TestUpdate_p(t *testing.T) {
	defer func() {
		if r := recover(); (reflect.TypeOf(r)).Name() == "MinMaxError" {
			fmt.Println("Recovered Error:", r)
		} else {
			t.Error(r)
		}
	}()
	for i, test := range tests {
		t.Logf("-- test #%d", i+1)
		c := NewPIDController(test.p, test.i, test.d)
		if test.min != 0 || test.max != 0 {
			c.SetOutputLimits(test.min, test.max)
		}
		for _, u := range test.updates {
			if err := u.check(c); err != nil {
				t.Error(err)
			}
		}
	}
}
