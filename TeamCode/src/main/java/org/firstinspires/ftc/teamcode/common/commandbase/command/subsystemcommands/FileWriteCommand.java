package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.FileInterface;

public class FileWriteCommand extends InstantCommand {
    public FileWriteCommand(String key, Object value) {
        super(
                () -> FileInterface.write(key, value.toString())
        );
    }
}
