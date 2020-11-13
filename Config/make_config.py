import json

config = {}
config['pose'] = {}
config['pose']['left'] = []
config['pose']['left'].append([0.5954899787902832, -2.141451498071188, -1.731623649597168, 2.683175726527832, -2.1759632269488733, 2.2040276527404785])
config['pose']['left'].append([0.6, -2.1, -1.7, 2.6, -2.1, 2.3])
config['pose']['left'].append([0.5, -5.0, 1.8, 4.4, -2.3, 2.6])
config['pose']['right'] = []
config['pose']['right'].append([5.728828430175781, -1.3247288030437012, 2.28007156053652, -3.235303064385885, -2.127599064503805, 1.2354044914245605])
config['pose']['right'].append([5.6, -1.4, 2.2, -3.2, -2.2, 1.3])
config['pose']['right'].append([6.0, -1.4, 2.2, -3.2, -2.2, 1.3])

config['ip'] = {}
config['ip']['left'] = "192.168.1.66"
config['ip']['right'] = "192.168.1.109"

with open("urx_config.json", 'w') as json_file:
    json.dump(config, json_file)