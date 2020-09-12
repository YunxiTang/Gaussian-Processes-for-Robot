%%% results of GP4DME (rand delete)
TIME = [10 20 30 40 50 60 70 80 90 100];
RMSE_PD = [0.6565 0.6847 0.6553 0.6694 0.6603 0.6676 0.6737 0.6648 0.6695 0.6633;
           0.6271 0.6322 0.6259 0.6236 0.6236 0.6278 0.6302 0.6286 0.6269 0.6260];
       
RMSE_CT = [7.0383 6.6404 6.4322 6.3605 6.3005 6.3672 6.3566 6.3192 6.3029 6.2771;
           2.1871 2.2126 2.1909 2.1872 2.1991 2.1749 2.1860 2.1725 2.1777 2.1867];
       
           
RMSE_GP_100 = [6.8529    6.0987    5.7089    5.5434    5.3730    5.2643    5.2058    5.1587    5.1101    5.0629;
               3.4374    2.8076    2.4948    2.3346    2.2156    2.1302    2.0740    2.0320    1.9965    1.9649];
RMSE_GP_200 = [8.1565    7.3610    6.5644    6.1092    5.8075    5.5990    5.4790    5.4011    5.3328    5.2675;
               3.5627    3.1025    2.7330    2.5209    2.3657    2.2544    2.1800    2.1270    2.0837    2.0455];
RMSE_GP_300 = [8.3075    7.5191    6.7511    6.2903    5.9671    5.7136    5.5471    5.4390    5.3295    5.2529;
               3.5310    3.0404    2.6884    2.4945    2.3491    2.2391    2.1615    2.1065    2.0578    2.0192];
RMSE_GP_400 = [8.0397    7.3718    6.6475    6.2278    5.9232    5.6858    5.5285    5.4119    5.2963    5.2114;
               3.3598    2.9822    2.6613    2.4881    2.3456    2.2322    2.1529    2.0918    2.0406    2.0001];
RMSE_GP_500 = [6.6087    6.4579    6.0486    5.8342    5.6521    5.4836    5.3843    5.2948    5.2085    5.1390;
               2.7703    2.6105    2.4092    2.3129    2.2188    2.1354    2.0811    2.0337    1.9921    1.9576];
RMSE_GP_600 = [5.6369    5.5228    5.2972    5.1521    5.0370    4.9397    4.9162    4.8890    4.8697    4.8440;
               2.1720    2.0909    1.9944    1.9387    1.8898    1.8482    1.8321    1.8174    1.8080    1.7954];
RMSE_GP_700 = [5.6089    5.4628    5.2359    5.0787    4.9787    4.8467    4.7964    4.7565    4.6999    4.6656;
               2.1643    2.0704    1.9732    1.9120    1.8692    1.8150    1.7894    1.7688    1.7475    1.7322];
RMSE_GP_800 = [5.6153    5.4546    5.2404    5.0921    4.9963    4.8792    4.8313    4.7906    4.7368    4.7036;
               2.1673    2.0686    1.9691    1.9139    1.8729    1.8260    1.8035    1.7843    1.7661    1.7522]; 
RMSE_GP_1000 = [5.6165    5.4622    5.2478    5.1125    5.0192    4.9079    4.8753    4.8356    4.7872    4.7541;
                2.1664    2.0697    1.9746    1.9248    1.8811    1.8355    1.8170    1.7980    1.7806    1.7662];
figure(1);
plot(TIME,RMSE_CT(1,:),'o-','color','r','linewidth',2.0);hold on;
plot(TIME,RMSE_GP_100(1,:),'s-','color',[0.1 0.1 0.8],'linewidth',2.0);hold on;
plot(TIME,RMSE_GP_200(1,:),'s-','color','g','linewidth',2.0);hold on;
plot(TIME,RMSE_GP_300(1,:),'s-','color',[0.8 0.1 0.1],'linewidth',2.0);hold on;
plot(TIME,RMSE_GP_400(1,:),'s-','color',[0.1 0.5 0.5],'linewidth',2.0);hold on;
plot(TIME,RMSE_GP_500(1,:),'s-','color',[0.8 0.2 0.5],'linewidth',2.0);hold on;
plot(TIME,RMSE_GP_600(1,:),'s-','color',[0.5 0.1 0.5],'linewidth',2.0);hold on;
plot(TIME,RMSE_GP_700(1,:),'s-','color',[0.5 0.6 0.3],'linewidth',2.0);hold on;
plot(TIME,RMSE_GP_800(1,:),'s-','color',[0.1 0.4 0.5],'linewidth',2.0);hold on;
plot(TIME,RMSE_GP_1000(1,:),'s-','color',[0.5 0.4 0.9],'linewidth',2.0);hold on;
legend('CT','GP_{100}','GP_{200}','GP_{300}','GP_{400}','GP_{500}','GP_{600}','GP_{700}','GP_{800}','GP_{1000}');
grid on;
title('Joint1');
xlabel('Training Time/s');
ylabel('RMSE/deg');
%%%
figure(2);
plot(TIME,RMSE_CT(2,:),'o-','color','r','linewidth',2.0);hold on;
plot(TIME,RMSE_GP_100(2,:),'s-','color',[0.1 0.1 0.8],'linewidth',2.0);hold on;
plot(TIME,RMSE_GP_200(2,:),'s-','color','g','linewidth',2.0);hold on;
plot(TIME,RMSE_GP_300(2,:),'s-','color',[0.8 0.1 0.1],'linewidth',2.0);hold on;
plot(TIME,RMSE_GP_400(2,:),'s-','color',[0.1 0.5 0.5],'linewidth',2.0);hold on;
plot(TIME,RMSE_GP_500(2,:),'s-','color',[0.8 0.2 0.5],'linewidth',2.0);hold on;
plot(TIME,RMSE_GP_600(2,:),'s-','color',[0.5 0.1 0.5],'linewidth',2.0);hold on;
plot(TIME,RMSE_GP_700(2,:),'s-','color',[0.5 0.6 0.3],'linewidth',2.0);hold on;
plot(TIME,RMSE_GP_800(2,:),'s-','color',[0.1 0.4 0.5],'linewidth',2.0);hold on;
plot(TIME,RMSE_GP_1000(2,:),'s-','color',[0.5 0.4 0.9],'linewidth',2.0);hold on;
legend('CT','GP_{100}','GP_{200}','GP_{300}','GP_{400}','GP_{500}','GP_{600}','GP_{700}','GP_{800}','GP_{1000}');
grid on;
title('Joint2');
xlabel('Training Time/s');
ylabel('RMSE/deg');