package com.acmerobotics.relicrecovery.configuration;

import android.app.Activity;
import android.os.Bundle;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.LinearLayout;
import android.widget.SeekBar;
import android.widget.Spinner;
import android.widget.TextView;

import com.acmerobotics.library.R;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.robotcore.internal.opmode.RegisteredOpModes;

import java.util.List;

public class OpModeConfigurationActivity extends Activity {

    private OpModeConfiguration configuration;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_opmode_configuration);

        configuration = new OpModeConfiguration(this);

        RegisteredOpModes registeredOpModes = RegisteredOpModes.getInstance();
        registeredOpModes.waitOpModesRegistered();
        List<OpModeMeta> opModeMetas = registeredOpModes.getOpModes();
        String[] opModes = new String[opModeMetas.size() + 1];
        opModes[0] = OpModeConfiguration.NO_AUTO_TRANSITION;
        String autoTransition = configuration.getAutoTransition();
        int selectedOpMode = 0;
        for (int i = 1; i < opModes.length; i++) {
            opModes[i] = opModeMetas.get(i - 1).name;
            if (opModes[i].equals(autoTransition)) {
                selectedOpMode = i;
            }
        }

        Spinner autoTransitionSpinner = (Spinner) findViewById(R.id.auto_transition_spinner);
        ArrayAdapter<String> adapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item, opModes);
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        autoTransitionSpinner.setAdapter(adapter);

        autoTransitionSpinner.setSelection(selectedOpMode);
        autoTransitionSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                configuration.setAutoTransition(opModes[position]);
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {

            }
        });

        final LinearLayout matchNumberLayout = (LinearLayout) findViewById(R.id.match_number_layout);

        FieldView fieldView = (FieldView) findViewById(R.id.field_view);
        fieldView.setConfiguration(configuration);

        Spinner matchTypeSpinner = (Spinner) findViewById(R.id.match_type_spinner);
        MatchType type = configuration.getMatchType();
        if (type == MatchType.PRACTICE) {
            matchNumberLayout.setVisibility(View.GONE);
        } else {
            matchNumberLayout.setVisibility(View.VISIBLE);
        }
        matchTypeSpinner.setSelection(type.getIndex());
        matchTypeSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                MatchType type = MatchType.fromIndex(position);
                if (type == MatchType.PRACTICE) {
                    matchNumberLayout.setVisibility(View.GONE);
                } else {
                    matchNumberLayout.setVisibility(View.VISIBLE);
                }
                configuration.setMatchType(type);
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {

            }
        });

        final TextView matchNumberValueTextView = (TextView) findViewById(R.id.match_number_value_text_view);

        SeekBar matchNumberSeekBar = (SeekBar) findViewById(R.id.match_number_seek_bar);
        int matchNumber = configuration.getMatchNumber();
        matchNumberSeekBar.setProgress(matchNumber - 1);
        matchNumberValueTextView.setText("#" + matchNumber);
        matchNumberSeekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                configuration.setMatchNumber(progress + 1);
                matchNumberValueTextView.setText("#" + (progress + 1));
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });

        final TextView delayValueTextView = (TextView) findViewById(R.id.delay_value_text_view);

        SeekBar delaySeekBar = (SeekBar) findViewById(R.id.delay_seek_bar);
        int delay = configuration.getDelay();
        delaySeekBar.setProgress(delay);
        delayValueTextView.setText(delay + "s");
        delaySeekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                configuration.setDelay(progress);
                delayValueTextView.setText(progress + "s");
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });

        Spinner balancingStoneSpinner = (Spinner) findViewById(R.id.balancing_stone_spinner);
        balancingStoneSpinner.setSelection(configuration.getBalancingStone().getIndex() - 2 * configuration.getAllianceColor().getIndex());
        balancingStoneSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                int index = position + 2 * configuration.getAllianceColor().getIndex();
                configuration.setBalancingStone(BalancingStone.fromIndex(index));
                fieldView.postInvalidate();
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {

            }
        });

        Spinner allianceColorSpinner = (Spinner) findViewById(R.id.alliance_color_spinner);
        allianceColorSpinner.setSelection(configuration.getAllianceColor().getIndex());
        allianceColorSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                configuration.setAllianceColor(AllianceColor.fromIndex(position));
                int index = balancingStoneSpinner.getSelectedItemPosition() + 2 * position;
                configuration.setBalancingStone(BalancingStone.fromIndex(index));
                fieldView.postInvalidate();
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {

            }
        });
    }
}