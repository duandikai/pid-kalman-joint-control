"""Independent checks of exported physical identities and reported metrics.

Usage: python verify_results.py [results_directory]
Requires Python 3 and NumPy, only for this optional verification tool.
The MATLAB simulation itself does not require Python.
"""
import csv
import sys
from pathlib import Path

import numpy as np


def close(a, b, label, atol=2e-10):
    if not np.allclose(a, b, atol=atol, rtol=2e-11, equal_nan=True):
        raise AssertionError(label)


def rows(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent / 'results'
data = root / 'data'
noise = np.genfromtxt(data / 'noise_standard.csv', delimiter=',', names=True)
config = {row['parameter']: float(row['value']) for row in rows(data / 'configuration.csv')}
dt = config['dt']
deg = 180 / np.pi
rms = lambda x: np.sqrt(np.mean(x*x))
expected_cases = {row['case']: row for row in rows(data / 'robustness_configuration.csv')}
files = [data / f'{kind}_{method}.csv' for kind in ('tracking','step') for method in ('encoder','lpf','kf')]
files += [data / f'robustness_{name}.csv' for name in expected_cases]
assert len(files) == 18

for path in files:
    z = np.genfromtxt(path, delimiter=',', names=True)
    assert len(z) == 5001, path
    assert all(np.isfinite(z[name]).all() for name in z.dtype.names), path
    t = z['time_s']
    close(t, np.arange(5001)*dt, f'{path.name}: time grid')
    q = t >= config['metric_start']
    is_step = path.stem.startswith('step_')
    ns = js = bs = 1
    if path.stem.startswith('robustness_'):
        case = expected_cases[path.stem[len('robustness_'):]]
        ns, js, bs = [float(case[key]) for key in ('noise_std_multiplier','plant_J_multiplier','plant_b_multiplier')]
    expected_angle = 30 + (0*t if is_step else 5*np.sin(2*np.pi*t))
    expected_rate = 0*t if is_step else 10*np.pi*np.cos(2*np.pi*t)
    close(z['reference_angle_deg'], expected_angle, 'Reference angle')
    close(z['reference_velocity_deg_s'], expected_rate, 'Reference derivative/units')
    close(z['encoder_angle_deg']-z['true_angle_deg'], ns*.2*noise['encoder_standard_normal'], 'Matched encoder noise')
    close(z['gyro_velocity_deg_s']-z['true_velocity_deg_s'], ns*.5*noise['gyro_standard_normal'], 'Matched gyro noise')
    close(z['tracking_error_deg'], expected_angle-z['true_angle_deg'], 'Tracking uses true angle')
    close(z['kf_angle_error_deg'], z['kf_angle_deg']-z['true_angle_deg'], 'Estimation uses true angle')
    close(z['applied_voltage_V'], np.clip(z['voltage_command_V'],-24,24), 'Applied voltage saturation')
    close(z['current_A'][1:], np.clip((z['applied_voltage_V'][1:]-z['true_velocity_deg_s'][:-1]/deg)/2,-10,10), 'Back EMF and current units')
    close(z['motor_torque_Nm'], z['current_A'], 'Torque constant')
    close(np.diff(z['true_angle_deg']),dt*z['true_velocity_deg_s'][1:], 'Semi-implicit kinematics')
    net = z['motor_torque_Nm']-z['friction_torque_Nm']-z['gravity_torque_Nm']-z['disturbance_torque_Nm']
    close(np.diff(z['true_velocity_deg_s'])/deg/dt,net[1:]/(.5*js),'Torque balance',atol=1e-8)
    close(z['gravity_torque_Nm'][1:],4.905*np.sin(z['true_angle_deg'][:-1]/deg),'Gravity torque')
    close(z['disturbance_torque_Nm'][1:],1.5*np.sin(10*np.pi*t[1:]),'External disturbance')
    method = path.stem.split('_')[-1] if not path.stem.startswith('robustness_') else 'kf'
    feedback_column = {'encoder':'encoder_angle_deg','lpf':'lpf_angle_deg','kf':'kf_angle_deg'}[method]
    close(z['feedback_used_deg'][1:],z[feedback_column][:-1],'Previous-sample feedback')
    close(z['controller_error_deg'][1:],expected_angle[1:]-z['feedback_used_deg'][1:],'Controller error timing')
    assert np.min(z['P11_rad2'])>0 and np.min(z['P22_rad2_per_s2'])>0
    assert np.min(z['P11_rad2']*z['P22_rad2_per_s2']-z['P12_rad2_per_s']**2)>-1e-18
    close(z['P11_deg2'],z['P11_rad2']*deg**2,'Covariance units')
    metrics = {row['metric']:float(row['value']) for row in rows(path.with_name(path.stem+'_metrics.csv'))}
    values = {
        'angle_rmse_deg':rms(z['tracking_error_deg'][q]),
        'velocity_rmse_deg_s':rms(z['velocity_tracking_error_deg_s'][q]),
        'angle_max_abs_deg':max(abs(z['tracking_error_deg'][q])),
        'velocity_max_abs_deg_s':max(abs(z['velocity_tracking_error_deg_s'][q])),
        'voltage_rms_V':rms(z['applied_voltage_V'][q]),
        'current_rms_A':rms(z['current_A'][q]),
        'encoder_rmse_deg':rms(z['encoder_error_deg'][q]),
        'gyro_rmse_deg_s':rms(z['gyro_error_deg_s'][q]),
        'kf_angle_rmse_deg':rms(z['kf_angle_error_deg'][q]),
        'kf_velocity_rmse_deg_s':rms(z['kf_velocity_error_deg_s'][q]),
        'encoder_rmse_full_deg':rms(z['encoder_error_deg']),
        'gyro_rmse_full_deg_s':rms(z['gyro_error_deg_s']),
        'kf_angle_rmse_full_deg':rms(z['kf_angle_error_deg']),
        'kf_velocity_rmse_full_deg_s':rms(z['kf_velocity_error_deg_s']),
        'settling_s':float('nan'),'overshoot_pct':float('nan'),
    }
    if is_step:
        violations=np.flatnonzero(np.abs(z['true_angle_deg']-30)>.6)
        values['settling_s']=0 if not len(violations) else (t[violations[-1]+1] if violations[-1]<len(t)-1 else float('nan'))
        values['overshoot_pct']=max(0,(max(z['true_angle_deg'])-30)/30*100)
    for key,value in values.items():
        close(metrics[key],value,f'{path.name}: {key}')

for row in rows(data / 'table2_controller_comparison.csv'):
    method=row['method']
    for field in list(row)[1:]:
        kind='step' if field in ('settling_s','overshoot_pct') else 'tracking'
        metrics={x['metric']:float(x['value']) for x in rows(data / f'{kind}_{method}_metrics.csv')}
        close(float(row[field]),metrics[field],'Table 2 traceability')
for row in rows(data / 'table3_robustness.csv'):
    metrics={x['metric']:float(x['value']) for x in rows(data / ('robustness_'+row['case']+'_metrics.csv'))}
    close(float(row['kf_angle_rmse_deg']),metrics['kf_angle_rmse_deg'],'Table 3 estimation')
    close(float(row['tracking_angle_rmse_deg']),metrics['angle_rmse_deg'],'Table 3 tracking')

z=np.genfromtxt(data/'tracking_kf.csv',delimiter=',',names=True)
q=z['time_s']>=1
x=np.column_stack([np.sin(2*np.pi*z['time_s'][q]),np.cos(2*np.pi*z['time_s'][q]),np.ones(q.sum())])
coeff=np.linalg.lstsq(x,np.column_stack([z['true_angle_deg'][q],z['kf_angle_deg'][q],z['lpf_angle_deg'][q]]),rcond=None)[0]
phases=np.arctan2(coeff[1],coeff[0])*deg
for i,row in enumerate(rows(data/'phase_comparison.csv'),1):
    expected=(phases[i]-phases[0]+180)%360-180
    close(float(row['signed_phase_difference_deg']),expected,'Phase regression')

audit=rows(data/'manuscript_comparison.csv')
for row in audit:
    computed=float(row['computed_value']); reported=float(row['manuscript_value'])
    close(computed-reported,float(row['computed_minus_manuscript']),'Audit difference')
    expected=np.isfinite(computed) and abs(computed-reported)<=float(row['rounding_tolerance'])+1e-12
    assert bool(int(row['matches_rounding']))==expected

print(f'PASS: {len(files)} trajectories, physical identities, matched noise, covariance, all metrics, Tables 2/3, phase calculations and audit flags.')
print(f'Manuscript agreement is a separate question: {sum(int(r["matches_rounding"]) for r in audit)}/{len(audit)} values match published rounding.')
