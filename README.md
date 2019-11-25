# as608 软件包

## 1、介绍

as608 软件包是对 AS60x 系列的指纹模块的支持

### 1.1 许可证

as608 package 遵循 LGPLv2.1 许可，详见 `LICENSE` 文件。

### 1.2 依赖

- RT-Thread 4.0+

## 2、如何打开 agile_button

使用 as608 package 需要先使用 `pkgs --upgrade` 更新包列表

然后在 RT-Thread 的包管理器中选择它，具体路径如下：

```
RT-Thread online packages
    peripheral libraries and drivers --->
        [*] AS608 fingerprint module driver  --->
            (136) AS608 WAK pin num
            [*]   Enable AS608 SAMPLE
```

AS608 WAK 管脚需要根据自己需要设置成相应的管脚

然后让 RT-Thread 的包管理器自动更新，或者使用 `pkgs --update` 命令更新包到 BSP 中。

## 3、示例说明

[samples.md](doc/samples.md)

## 4、联系方式 & 感谢

* 维护：greedyhao
* 主页：<https://github.com/greedyhao>
* 邮箱：<hao_kr@163.com>
