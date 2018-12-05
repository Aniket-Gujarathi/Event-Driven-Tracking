root_dir = 'C:\Users\AGlover\Documents\workspace\dump\trackingcomparison2\';
dataset_dir = '150_512';

raw_file = [root_dir dataset_dir '\ATIS\data.log.txt'];
gt_file = [root_dir dataset_dir '\ATIS\data.log.GT'];
cpu_file = [root_dir dataset_dir '\cpu\data.log.txt'];
spin_file = [root_dir dataset_dir '\spinnaker\data.log.txt'];
latency_file = [root_dir dataset_dir '\latency.txt'];
accuracy_file = [root_dir dataset_dir '\accuracy.txt'];
stats_file = [root_dir dataset_dir '\data_stats.txt'];

time_period = [0 30];
resolution = 0.01; %of latency measurements
PUBLISH = false;

c = hot(3) * 0.85;
c1 = c(1, :);
c2 = c(2, :);
c3 = c(3, :);

infilename = [root_dir dataset_dir '\ATIS_00001\data.log'];
GTdataset = [infilename '.txt'];
CODEC_TYPE = 2;
TIMESTAMP_BITS = 30;
CLOCK_PERIOD = 0.00000008;